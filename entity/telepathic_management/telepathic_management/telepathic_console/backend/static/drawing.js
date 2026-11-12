/* Retained drawing objects allow both Grace and the user to edit the same artwork. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const canvas = $('drawingCanvas'), ctx = canvas.getContext('2d');
  const WIDTH = 1000, HEIGHT = 650, KEY = 'grace-canvas-v1';
  const drawingKinds = new Set(['path','curve','polygon','erase','line','rectangle','ellipse','text','letter']);
  const kinds = new Set([...drawingKinds,'select','move','delete','style','front','back','clear','undo','redo']);
  const fonts = {sans:'system-ui, sans-serif',serif:'Georgia, serif',mono:'monospace',handwritten:'"Segoe Print", "Bradley Hand", cursive'};
  let objects = [], undo = [], redo = [], selected = null, gesture = null;
  let busy = false, connected = false, token = 0, requestId = null, deadline = null;
  let inputTopic, replay = Promise.resolve();
  const clone = value => JSON.parse(JSON.stringify(value));
  const status = text => { $('drawStatus').textContent = text; };
  const uid = () => globalThis.crypto?.randomUUID?.() || `shape-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  const fontFamily = a => fonts[a.font] || (a.kind==='letter' ? fonts.serif : fonts.sans);
  function save() { try { localStorage.setItem(KEY, JSON.stringify(objects)); } catch (_) { status('Browser storage is full; use Save PNG to keep your artwork.'); } }
  function checkpoint(snapshot = objects) { undo.push(clone(snapshot)); if (undo.length > 50) undo.shift(); redo = []; }
  function style(a) {
    ctx.strokeStyle=a.color||'#c75b4b';ctx.fillStyle=a.fill_color||a.color||'#c75b4b';
    ctx.lineWidth=Math.max(.5,a.size??5);ctx.lineCap=ctx.lineJoin='round';
    ctx.globalAlpha=(a.opacity??1)*(a.brush==='marker'?.4:1);
    if(a.brush==='soft'){ctx.shadowColor=a.color||'#c75b4b';ctx.shadowBlur=Math.min(60,(a.size||5)*.8);}
    if(a.gradient && a.gradient!=='none' && (a.gradient_color || a.gradient_stops)){
      const [x,y,w,h]=bounds(a),radius=Math.max(1,w,h)/2;
      const angle=(a.gradient_angle??45)*Math.PI/180, vx=Math.cos(angle), vy=Math.sin(angle);
      const extent=Math.max(1,Math.abs(w*vx)+Math.abs(h*vy))/2, cx=x+w/2, cy=y+h/2;
      const gradient=a.gradient==='radial'?ctx.createRadialGradient(x+w*.4,y+h*.35,0,cx,cy,radius):ctx.createLinearGradient(cx-vx*extent,cy-vy*extent,cx+vx*extent,cy+vy*extent);
      const stops=a.gradient_stops || [{offset:0,color:a.fill_color||a.color||'#c75b4b'},{offset:1,color:a.gradient_color}];
      stops.forEach(stop=>gradient.addColorStop(stop.offset,stop.color));ctx.fillStyle=gradient;
    }
  }
  // Lay out the complete letter before replay so wrapping stays stable as text appears.
  function letterLayout(a) {
    const x=a.x??270, y=a.y??20, width=a.width??460, height=a.height??610;
    const margin=a.margin??40, size=Math.max(8,a.size??16), step=size*(a.line_height??1.5);
    const available=width-2*margin, lines=[];
    if(available<size || height-2*margin<step || x+width>WIDTH || y+height>HEIGHT) throw new Error('Letter page does not fit. Adjust its size, position, or margins.');
    ctx.save(); ctx.font=`${size}px ${fontFamily(a)}`;
    for(const paragraph of (a.text||'').split('\n')) {
      let line='';
      for(const word of paragraph.trim().split(/\s+/).filter(Boolean)) {
        if(line && ctx.measureText(`${line} ${word}`).width>available) { lines.push(line);line=''; }
        // Split a long word too, so addresses and URLs cannot escape the margins.
        for(const character of Array.from((line?' ':'')+word)) {
          if(ctx.measureText(line+character).width>available && line) { lines.push(line);line=''; }
          line+=character;
        }
      }
      lines.push(line);
    }
    ctx.restore();
    if(lines.length*step>height-2*margin) throw new Error('Letter is too long for one page. Shorten it, reduce its font size, or reduce spacing or margins.');
    return {x,y,width,height,margin,size,step,lines};
  }
  function textLayout(a) {
    const size=Math.max(8,Math.min(120,a.size||24));
    const x=Math.max(12,Math.min(WIDTH-80,a.x??12));
    const y=Math.max(12,Math.min(HEIGHT-size-12,a.y??12));
    const maxWidth=WIDTH-12-x, maxHeight=HEIGHT-12-y, lineHeight=size*1.2, lines=[];
    ctx.font=`${size}px ${fontFamily(a)}`;
    const pushWrapped = source => {
      let line='';
      for(const word of source.split(/\s+/).filter(Boolean)) {
        const candidate=line?`${line} ${word}`:word;
        if(ctx.measureText(candidate).width<=maxWidth) { line=candidate; continue; }
        if(line) { lines.push(line); line=''; }
        for(const character of Array.from(word)) {
          if(line&&ctx.measureText(line+character).width>maxWidth) { lines.push(line);line=''; }
          line+=character;
        }
      }
      lines.push(line);
    };
    (a.text||'').split('\n').forEach(pushWrapped);
    const maxLines=Math.max(1,Math.floor(maxHeight/lineHeight));
    if(lines.length>maxLines) {
      lines.length=maxLines;
      let last=lines[maxLines-1];
      while(last&&ctx.measureText(`${last}…`).width>maxWidth) last=Array.from(last).slice(0,-1).join('');
      lines[maxLines-1]=`${last}…`;
    }
    return {x,y,size,lineHeight,lines,width:Math.max(0,...lines.map(line=>ctx.measureText(line).width)),height:lines.length*lineHeight};
  }
  function paint(a, progress = 1) {
    ctx.save(); style(a);
    if (a.kind === 'erase') ctx.globalCompositeOperation = 'destination-out';
    if (a.kind === 'curve') {
      const points=a.points, count=(points.length-1)/3, travel=count*progress;
      ctx.beginPath();ctx.moveTo(...points[0]);
      for(let i=0;i<count && i<travel;i++) {
        const start=points[i*3], p1=points[i*3+1], p2=points[i*3+2], end=points[i*3+3];
        const t=Math.min(1,travel-i);
        // de Casteljau subdivision preserves the curve while revealing a partial segment.
        const lerp=(p,q)=>[p[0]+(q[0]-p[0])*t,p[1]+(q[1]-p[1])*t];
        const q0=lerp(start,p1), q1=lerp(p1,p2), q2=lerp(p2,end);
        const r0=lerp(q0,q1), r1=lerp(q1,q2), tip=lerp(r0,r1);
        ctx.bezierCurveTo(...q0,...r0,...tip);
      }
      if(a.fill && progress===1){ctx.closePath();ctx.fill();}
      if(a.stroke!==false)ctx.stroke();
    } else if (['path','polygon','erase','line'].includes(a.kind)) {
      const points = a.points || []; if (!points.length) { ctx.restore(); return; }
      const travel = Math.max(0, (points.length - 1) * progress), whole = Math.floor(travel);
      ctx.beginPath(); ctx.moveTo(...points[0]);
      for (let i = 1; i <= whole; i++) {
        if(a.smooth && i<whole)ctx.quadraticCurveTo(...points[i],(points[i][0]+points[i+1][0])/2,(points[i][1]+points[i+1][1])/2);
        else ctx.lineTo(...points[i]);
      }
      if (whole < points.length - 1) { const f=travel-whole;ctx.lineTo(points[whole][0]+(points[whole+1][0]-points[whole][0])*f,points[whole][1]+(points[whole+1][1]-points[whole][1])*f); }
      if (points.length === 1) ctx.lineTo(points[0][0]+.01, points[0][1]);
      if((a.fill||a.kind==='polygon') && progress===1){ctx.closePath();ctx.fill();}
      if(a.stroke!==false||a.kind==='erase')ctx.stroke();
    } else if (a.kind === 'letter') {
      const page=letterLayout(a), {x,y,width,height,margin,size,step,lines}=page;
      ctx.fillStyle='#ffffff';ctx.fillRect(x,y,width,height);
      ctx.strokeStyle='#b6c4d3';ctx.lineWidth=1;ctx.strokeRect(x,y,width,height);
      ctx.font=`${size}px ${fontFamily(a)}`;ctx.textBaseline='top';ctx.textAlign=a.align||'left';ctx.fillStyle=a.color||'#34313d';
      const left=x+margin, right=x+width-margin, anchor=a.align==='center'?(left+right)/2:a.align==='right'?right:left;
      let remaining=Math.ceil(lines.reduce((n,line)=>n+Array.from(line).length,0)*progress);
      lines.forEach((line,i)=>{const chars=Array.from(line);ctx.fillText(chars.slice(0,Math.max(0,remaining)).join(''),anchor,y+margin+i*step);remaining-=chars.length;});
    } else if (a.kind === 'text') {
      const layout=textLayout(a), chars=Math.ceil(Array.from(a.text||'').length*progress);
      ctx.font=`${layout.size}px ${fontFamily(a)}`;ctx.textBaseline='top';ctx.textAlign=a.align||'left';
      let remaining=chars;
      const anchor=layout.x+(a.align==='center'?layout.width/2:a.align==='right'?layout.width:0);
      layout.lines.forEach((line,i)=>{const visible=Array.from(line).slice(0,Math.max(0,remaining)).join('');ctx.fillText(visible,anchor,layout.y+i*layout.lineHeight);remaining-=Array.from(line).length;});
    } else {
      ctx.beginPath();
      if (a.kind === 'ellipse') ctx.ellipse((a.x||0)+(a.width||0)/2,(a.y||0)+(a.height||0)/2,Math.abs(a.width||0)/2,Math.abs(a.height||0)/2,0,-Math.PI/2,-Math.PI/2+Math.PI*2*progress);
      else if (a.kind === 'rectangle') {
        const x=a.x||0,y=a.y||0,w=a.width||0,h=a.height||0, pts=[[x,y],[x+w,y],[x+w,y+h],[x,y+h],[x,y]];
        ctx.moveTo(x,y); const edges=progress*4;
        for(let i=0;i<4;i++) { if(edges>=i+1) ctx.lineTo(...pts[i+1]); else if(edges>i) ctx.lineTo(pts[i][0]+(pts[i+1][0]-pts[i][0])*(edges-i),pts[i][1]+(pts[i+1][1]-pts[i][1])*(edges-i)); }
      }
      if (a.fill && progress === 1) ctx.fill(); if(a.stroke!==false)ctx.stroke();
    }
    ctx.restore();
  }
  function bounds(a) {
    if(a.kind==='letter') { const p=letterLayout(a);return [p.x,p.y,p.width,p.height]; }
    if (a.points) { const xs=a.points.map(p=>p[0]), ys=a.points.map(p=>p[1]); return [Math.min(...xs),Math.min(...ys),Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys)]; }
    if(a.kind==='text') { const p=textLayout(a);return [p.x,p.y,p.width,p.height]; }
    return [a.x||0,a.y||0,a.width||0,a.height||0];
  }
  function render(preview = null, progress = 1, selection = true) {
    ctx.clearRect(0,0,WIDTH,HEIGHT); objects.forEach(a=>paint(a));
    if (preview) paint(preview,progress);
    const active=objects.find(a=>a.id===selected);
    if(selection && active) { ctx.save(); ctx.strokeStyle='#bc688c';ctx.lineWidth=1.5;ctx.setLineDash([5,4]);const [x,y,w,h]=bounds(active);ctx.strokeRect(x-4,y-4,w+8,h+8);ctx.restore(); }
    $('drawLetter').textContent=active?.kind==='letter'?'Update letter page':'Add letter page';
  }
  function editLetter() {
    const a=objects.find(o=>o.id===selected);if(!a)return;
    $('drawColor').value=a.color||'#c75b4b';$('fillColor').value=a.fill_color||a.color||'#c75b4b';
    $('drawOpacity').value=(a.opacity??1)*100;$('drawBrush').value=a.brush||'ink';
    $('drawGradient').value=a.gradient||'none';$('gradientColor').value=a.gradient_color||'#f4c56a';
    $('gradientAngle').value=a.gradient_angle??45;$('drawFont').value=a.font||(a.kind==='letter'?'serif':'sans');
    $('drawSmooth').checked=a.smooth??true;$('drawSize').value=a.size??5;
    $('drawFill').checked=a.fill===true||a.kind==='polygon';$('drawStroke').checked=a.stroke!==false;
    if(a.kind==='text')$('drawText').value=a.text||'';
    if(a.kind!=='letter')return;
    $('drawText').value=a.text||'';$('drawSize').value=a.size??16;$('drawColor').value=a.color||'#183b65';
    $('letterAlign').value=a.align||'left';$('letterMargin').value=a.margin??40;$('letterSpacing').value=String(a.line_height??1.5);
  }
  let layerSignature='';
  function syncControls() {
    $('emptyCanvasState').hidden=objects.length>0||busy;
    $('objectCount').textContent=`${objects.length} / 100`;
    $('drawUndo').disabled=busy||!undo.length;$('drawRedo').disabled=busy||!redo.length;
    for(const id of ['drawDelete','drawFront','drawBack','applyStyle'])$(id).disabled=busy||!selected;
    const signature=JSON.stringify([selected,busy,objects.map(a=>[a.id,a.kind,a.fill_color,a.color,a.text])]);
    if(signature===layerSignature)return;
    layerSignature=signature;
    const fragment=document.createDocumentFragment();
    if(!objects.length) {
      const note=document.createElement('p');note.className='layers-empty';note.textContent='Your marks and Grace’s strokes will appear here.';fragment.append(note);
    }
    [...objects].reverse().forEach((a,index)=>{
      const button=document.createElement('button');button.type='button';button.className='layer-item';
      button.disabled=busy;button.setAttribute('aria-pressed',String(a.id===selected));
      const swatch=document.createElement('span');swatch.className='layer-swatch';swatch.style.background=a.fill_color||a.color||'#c75b4b';
      const label=document.createElement('span');label.className='layer-label';
      label.textContent=a.text?.slice(0,36)||(a.id?.startsWith('shape-')||/^[\da-f]{8}-/i.test(a.id)?`${a.kind} ${objects.length-index}`:a.id)||a.kind;
      button.title=`Select ${label.textContent}`;button.append(swatch,label);
      button.addEventListener('click',()=>{if(busy)return;finishGesture();action({kind:'select',id:a.id});$('drawTool').value='select';canvas.style.cursor='move';});
      fragment.append(button);
    });
    $('layerList').replaceChildren(fragment);
  }
  function move(a,dx,dy) {
    const [bx,by,bw,bh]=bounds(a);
    dx=Math.max(-bx,Math.min(WIDTH-bx-bw,dx));
    dy=Math.max(-by,Math.min(HEIGHT-by-bh,dy));
    if(a.points) a.points=a.points.map(([x,y])=>[x+dx,y+dy]);
    else if(a.kind==='letter') { a.x=Math.max(0,Math.min(WIDTH-a.width,a.x+dx));a.y=Math.max(0,Math.min(HEIGHT-a.height,a.y+dy)); }
    else { a.x=(a.x||0)+dx;a.y=(a.y||0)+dy; }
  }
  function action(a) {
    if(a.kind==='select') { if(!objects.some(o=>o.id===a.id)) throw new Error(`Object not found: ${a.id}`); selected=a.id;editLetter(); }
    else if(a.kind==='undo') { if(undo.length) { redo.push(clone(objects));objects=undo.pop();selected=null; } }
    else if(a.kind==='redo') { if(redo.length) { undo.push(clone(objects));objects=redo.pop();selected=null; } }
    else if(a.kind==='clear') { checkpoint();objects=[];selected=null; }
    else if(['style','front','back'].includes(a.kind)) {
      const target=objects.find(o=>o.id===(a.id||selected));if(!target)throw new Error('Select an object first.');
      if(a.kind==='style') {
        const revised=normalize({...target,...a,kind:target.kind,id:target.id});
        if(revised.kind==='letter')letterLayout(revised);
        checkpoint();
        for(const key of ['color','fill_color','gradient_color','gradient','gradient_angle','gradient_stops','opacity','brush','stroke','fill','smooth','size','font','align']){if(key in a)target[key]=a[key];}
        if(('gradient_color' in a || 'fill_color' in a) && !('gradient_stops' in a))delete target.gradient_stops;
      }
      else {checkpoint();objects=objects.filter(o=>o!==target);a.kind==='front'?objects.push(target):objects.unshift(target);}
    }
    else if(['move','delete'].includes(a.kind)) {
      const id=a.id||selected, target=objects.find(o=>o.id===id);if(!target) throw new Error('Select an object first.');
      checkpoint();if(a.kind==='delete') { objects=objects.filter(o=>o.id!==id);selected=null; } else move(target,a.dx||0,a.dy||0);
    } else {
      if(a.kind==='letter') letterLayout(a);
      if(objects.length>=100) throw new Error('Canvas has 100 objects. Delete some before adding more.');
      if(a.id && objects.some(o=>o.id===a.id)) throw new Error(`Object ID already exists: ${a.id}`);
      checkpoint(); objects.push({...a,id:a.id||uid()}); selected=null;
    }
    render();save();syncControls();
  }
  function setBusy(value) {
    busy=value; $('drawAsk').disabled=value||!connected; $('drawStop').disabled=!value;
    document.querySelectorAll('.inspector input,.inspector select,.inspector textarea,.inspector button,.history-actions button,#importProject').forEach(el=> { if(el.id!=='drawStop' && el.id!=='drawExport') el.disabled=value; });
    syncControls();
  }
  async function animate(a, session) {
    if(['select','move','delete','style','front','back','clear','undo','redo'].includes(a.kind)) { action(a); return; }
    if(a.kind==='letter') letterLayout(a);
    const duration=window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 0 : (['text','letter'].includes(a.kind) ? Math.min(6000,(a.text||'').length*40) : 600);
    // Paint completed layers once; only the new stroke needs redrawing every frame.
    render(null,1,false);
    const base=document.createElement('canvas');base.width=WIDTH;base.height=HEIGHT;
    base.getContext('2d').drawImage(canvas,0,0);
    const start=performance.now();
    while(duration && performance.now()-start<duration) {
      if(session!==token) { render();return; }
      ctx.clearRect(0,0,WIDTH,HEIGHT);ctx.drawImage(base,0,0);paint(a,(performance.now()-start)/duration);
      await new Promise(requestAnimationFrame);
    }
    if(session===token) action(a);
  }
  function normalize(a) {
    if(!a || !kinds.has(a.kind)) throw new Error('Unknown drawing action');
    const out={kind:a.kind};
    for(const key of ['x','y','width','height','dx','dy','size','margin','line_height','opacity','gradient_angle']) if(key in a) { if(typeof a[key]!=='number'||!Number.isFinite(a[key])) throw new Error('Invalid drawing coordinates');const max={x:WIDTH,y:HEIGHT,width:WIDTH,height:HEIGHT,dx:WIDTH,dy:HEIGHT,size:120,margin:100,line_height:2,opacity:1,gradient_angle:360}[key];out[key]=Math.max(key==='dx'||key==='dy'?-max:key==='line_height'?1:0,Math.min(max,a[key])); }
    if('align' in a) { if(!['left','center','right'].includes(a.align)) throw new Error('Invalid letter alignment');out.align=a.align; }
    for(const key of ['text','id']) if(key in a) { if(typeof a[key]!=='string'||a[key].length>(key==='text'?2000:64)) throw new Error('Invalid drawing text');out[key]=a[key]; }
    for(const key of ['color','fill_color','gradient_color'])if(key in a){if(!/^#[0-9a-f]{6}$/i.test(a[key]))throw new Error('Invalid color');out[key]=a[key];}
    for(const [key,choices] of [['brush',['ink','soft','marker']],['gradient',['none','linear','radial']],['font',Object.keys(fonts)]])if(key in a){if(!choices.includes(a[key]))throw new Error(`Invalid ${key}`);out[key]=a[key];}
    if('gradient_stops' in a) {
      if(!Array.isArray(a.gradient_stops)||a.gradient_stops.length<2||a.gradient_stops.length>8)throw new Error('Expected 2–8 gradient stops');
      let previous=-1;
      out.gradient_stops=a.gradient_stops.map(stop=>{
        if(!stop||typeof stop.offset!=='number'||!Number.isFinite(stop.offset)||stop.offset<0||stop.offset>1||stop.offset<previous||!/^#[0-9a-f]{6}$/i.test(stop.color))throw new Error('Invalid gradient stop');
        previous=stop.offset;return {offset:stop.offset,color:stop.color};
      });
    }
    for(const key of ['stroke','smooth'])if(key in a){if(typeof a[key]!=='boolean')throw new Error('Invalid brush option');out[key]=a[key];}
    if(a.kind!=='style'||'fill' in a)out.fill=a.fill===true;
    if(['line','path','polygon','erase','curve'].includes(a.kind)) { if(!Array.isArray(a.points)||!a.points.length||a.points.length>2000) throw new Error('Invalid drawing path');out.points=a.points.map(p=>{if(!Array.isArray(p)||p.length!==2||p.some(v=>typeof v!=='number'||!Number.isFinite(v))) throw new Error('Invalid path point');return [Math.max(0,Math.min(WIDTH,p[0])),Math.max(0,Math.min(HEIGHT,p[1]))];}); }
    if(a.kind==='curve' && (out.points.length<4 || out.points.length>61 || (out.points.length-1)%3))throw new Error('A curve needs a start followed by groups of two control points and an endpoint.');
    if(['rectangle','ellipse'].includes(out.kind)) {
      out.width=Math.min(out.width??0,WIDTH);out.height=Math.min(out.height??0,HEIGHT);
      out.x=Math.min(out.x??0,WIDTH-out.width);out.y=Math.min(out.y??0,HEIGHT-out.height);
    }
    if(out.kind==='letter') {
      Object.assign(out,{x:out.x??270,y:out.y??20,width:out.width??460,height:out.height??610,size:out.size??16});
      out.width=Math.min(out.width,WIDTH);out.height=Math.min(out.height,HEIGHT);
      out.x=Math.min(out.x,WIDTH-out.width);out.y=Math.min(out.y,HEIGHT-out.height);
      letterLayout(out);
    }
    return out;
  }
  function point(e) { const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(WIDTH,(e.clientX-r.left)*WIDTH/r.width)),Math.max(0,Math.min(HEIGHT,(e.clientY-r.top)*HEIGHT/r.height))]; }
  canvas.addEventListener('pointerdown',e=> {
    if(busy || gesture || e.button!==0 || e.isPrimary===false) return; e.preventDefault();canvas.focus({preventScroll:true});canvas.setPointerCapture(e.pointerId);
    const p=point(e), kind=$('drawTool').value;
    if(kind==='select') { selected=null;for(let i=objects.length-1;i>=0;i--) { const a=objects[i];if(a.kind==='erase') continue; const [x,y,w,h]=bounds(a);if(p[0]>=x-8&&p[0]<=x+w+8&&p[1]>=y-8&&p[1]<=y+h+8){selected=a.id;break;} } editLetter();gesture={kind,start:p,original:clone(objects)};render();syncControls();return; }
    const a={kind,smooth:$('drawSmooth').checked,fill_color:$('fillColor').value,gradient_color:$('gradientColor').value,gradient:$('drawGradient').value,gradient_angle:Number($('gradientAngle').value)||0,font:$('drawFont').value,align:$('letterAlign').value,opacity:Number($('drawOpacity').value)/100,brush:$('drawBrush').value,stroke:$('drawStroke').checked,color:$('drawColor').value,size:Math.max(1,Math.min(120,Number($('drawSize').value)||5)),fill:kind==='polygon'||$('drawFill').checked};
    if(kind==='text') { a.text=$('drawText').value; a.x=p[0];a.y=p[1];if(a.text) action(a);return; }
    if(['rectangle','ellipse'].includes(kind))Object.assign(a,{x:p[0],y:p[1],width:0,height:0});else a.points=[p];
    gesture={kind,start:p,object:a};
  });
  canvas.addEventListener('pointermove',e=> {
    if(!gesture || busy) return;const p=point(e),g=gesture;
    if(g.kind==='select') { if(!selected) return;objects=clone(g.original);move(objects.find(a=>a.id===selected),p[0]-g.start[0],p[1]-g.start[1]);render();return; }
    const a=g.object;if(['rectangle','ellipse'].includes(a.kind)) { a.x=Math.min(g.start[0],p[0]);a.y=Math.min(g.start[1],p[1]);a.width=Math.abs(p[0]-g.start[0]);a.height=Math.abs(p[1]-g.start[1]); }
    else if(a.kind==='line') a.points=[g.start,p];else if(a.points.length<2000) a.points.push(p);
    render(a);
  });
  function finishGesture(cancel=false) {
    if(!gesture)return;const g=gesture;gesture=null;
    if(g.kind==='select') { if(cancel)objects=g.original;else if(selected&&JSON.stringify(objects)!==JSON.stringify(g.original)){checkpoint(g.original);save();} }
    else if(!cancel) { try{action(normalize(g.object));}catch(e){status(e.message);} }render();syncControls();
  }
  const palettes={
    studio:['#282532','#fffaf0','#c05d78','#e79c81','#edca83','#f4e6bd','#8f9f7a','#365e56','#6eada5','#8392bf','#71608c','#ad8baf','#dfb3bc','#a07057','#7e7267','#cbd0bd'],
    botanical:['#263e36','#48634a','#72845b','#9eae7c','#c6cda3','#efe8c7','#be754f','#df9b6e','#a94865','#d37d94','#eebac6','#654560','#93729c','#cab6ca','#f7eee2','#343631'],
    sunset:['#3e3656','#61517d','#93789c','#bda0b4','#dfbac4','#f3d1bd','#f9dfb2','#efb477','#dd895e','#ba6257','#8f495a','#553c52','#617d82','#9aa9a0','#e7e0cd','#fff5de'],
    graphite:['#202126','#363840','#52545d','#737681','#9c9ea8','#c5c6ce','#ecebf0','#ffffff','#463b3a','#796762','#a99485','#d6c5b5','#374643','#72817c','#aebbb1','#e6ebe1']
  };
  function renderPalette(){
    $('colorPalette').replaceChildren();
    (palettes[$('palettePreset').value]||palettes.studio).forEach(color=>{
      const button=document.createElement('button');button.type='button';button.style.background=color;
      button.title=`Use ${color}`;button.setAttribute('aria-label',`Use ${color}`);
      button.addEventListener('click',()=>{if(!busy)$($('paletteTarget').value==='fill'?'fillColor':'drawColor').value=color;});
      $('colorPalette').append(button);
    });
  }
  renderPalette();$('palettePreset').addEventListener('change',renderPalette);
  $('applyStyle').addEventListener('click',()=>{if(busy)return;try{action({kind:'style',color:$('drawColor').value,fill_color:$('fillColor').value,gradient_color:$('gradientColor').value,gradient:$('drawGradient').value,gradient_angle:Number($('gradientAngle').value)||0,opacity:Number($('drawOpacity').value)/100,brush:$('drawBrush').value,fill:$('drawFill').checked,stroke:$('drawStroke').checked,smooth:$('drawSmooth').checked,size:Math.max(1,Math.min(120,Number($('drawSize').value)||5)),font:$('drawFont').value,align:$('letterAlign').value});}catch(error){status(error.message);}});
  for(const kind of ['front','back'])$(kind==='front'?'drawFront':'drawBack').addEventListener('click',()=>{if(!busy)try{action({kind});}catch(error){status(error.message);}});
  $('drawTool').addEventListener('change',()=>{if($('drawTool').value==='text'){$('drawSize').value=24;document.querySelector('.writing-tools').open=true;}canvas.style.cursor=$('drawTool').value==='select'?'move':$('drawTool').value==='text'?'text':'crosshair';});
  $('drawLetter').addEventListener('click',()=> {
    if(busy)return;
    const current=objects.find(a=>a.id===selected&&a.kind==='letter');
    try {
      const a=normalize({...current,kind:'letter',text:$('drawText').value,color:$('drawColor').value,font:$('drawFont').value,
        size:current?Number($('drawSize').value):16,align:$('letterAlign').value,
        margin:Number($('letterMargin').value),line_height:Number($('letterSpacing').value)});
      if(!a.text.trim())throw new Error('Enter your letter in the Text field first.');
      if(current) { checkpoint();objects[objects.indexOf(current)]=a;render();save();syncControls(); }
      else action(a);
      status('Letter page saved. Use Select / move to edit it, or Save PNG to export it.');
    }catch(error){status(error.message);}
  });
  canvas.addEventListener('pointerup',()=>finishGesture());canvas.addEventListener('pointercancel',()=>finishGesture(true));
  for(const [id,kind] of [['drawUndo','undo'],['drawRedo','redo'],['drawDelete','delete'],['drawClear','clear']]) $(id).addEventListener('click',()=>{if(!busy)try{action({kind});}catch(e){status(e.message);}});
  canvas.addEventListener('keydown',e=>{if(!busy&&(e.key==='Delete'||e.key==='Backspace')){e.preventDefault();try{action({kind:'delete'});}catch(error){status(error.message);}}});
  document.addEventListener('keydown',e=>{
    if(busy||e.target.closest('input,textarea,select,[contenteditable]'))return;
    const key=e.key.toLowerCase();
    if((e.ctrlKey||e.metaKey)&&key==='z'){e.preventDefault();finishGesture(true);action({kind:e.shiftKey?'redo':'undo'});}
    else if(!e.ctrlKey&&!e.metaKey&&!e.altKey){
      const tool={b:'path',e:'erase',v:'select',l:'line',r:'rectangle',o:'ellipse',t:'text'}[key];
      if(tool){$('drawTool').value=tool;$('drawTool').dispatchEvent(new Event('change'));}
    }
  });
  $('drawExport').addEventListener('click',()=>{render(null,1,false);const output=document.createElement('canvas');output.width=WIDTH;output.height=HEIGHT;const exportCtx=output.getContext('2d');exportCtx.fillStyle='white';exportCtx.fillRect(0,0,WIDTH,HEIGHT);exportCtx.drawImage(canvas,0,0);const link=document.createElement('a');link.download='grace-drawing.png';link.href=output.toDataURL('image/png');link.click();render();});
  function documentObjects(source) {
    if(!Array.isArray(source)||source.length>100)throw new Error('Artwork must contain at most 100 layers.');
    const ids=new Set();
    return source.map(item=>{
      const a=normalize(item);
      if(!drawingKinds.has(a.kind))throw new Error('An artwork file may only contain drawing layers.');
      a.id=a.id||uid();
      if(ids.has(a.id))throw new Error('Artwork contains duplicate layer IDs.');
      ids.add(a.id);return a;
    });
  }
  function replaceArtwork(document) {
    if(!document||document.version!==1)throw new Error('Unsupported artwork file. Open a Grace artwork JSON file.');
    const next=documentObjects(document.objects);finishGesture(true);checkpoint();objects=next;selected=null;
    render();save();syncControls();
  }
  $('saveProject').addEventListener('click',()=>{
    const blob=new Blob([JSON.stringify({version:1,objects},null,2)],{type:'application/json'});
    const url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download='grace-artwork.json';link.click();
    setTimeout(()=>URL.revokeObjectURL(url),1000);status('Editable artwork saved. Use Open artwork to continue later.');
  });
  $('importProject').addEventListener('change',async e=>{
    const file=e.target.files?.[0];e.target.value='';if(!file||busy)return;
    const session=++token;
    setBusy(true);
    try{
      if(file.size>2*1024*1024)throw new Error('Choose an artwork file smaller than 2 MB.');
      const document=JSON.parse(await file.text());
      if(session!==token)return;
      replaceArtwork(document);status('Artwork opened. Undo restores your previous canvas.');
    }catch(error){if(session===token)status(error.message);}
    finally{if(session===token)setBusy(false);}
  });
  $('loadStudy').addEventListener('click',async()=>{
    if(busy)return;const session=++token;setBusy(true);
    try{
      const response=await fetch('/static/canvas-study.json');
      if(!response.ok)throw new Error('The colour study could not be loaded.');
      const document=await response.json();if(session!==token)return;
      replaceArtwork(document);status('Colour study opened — an editable example. Undo restores your previous canvas.');
      $('drawResponse').textContent='A prepared colour study to explore. Select any layer to change its colour, or ask Grace to develop the scene.';
    }catch(error){if(session===token)status(error.message);}
    finally{if(session===token)setBusy(false);}
  });
  document.querySelectorAll('[data-prompt]').forEach(button=>button.addEventListener('click',()=>{
    $('drawPrompt').value=button.dataset.prompt;$('drawPrompt').focus();
  }));
  $('drawOpacity').addEventListener('input',()=>{if($('opacityValue'))$('opacityValue').textContent=`${$('drawOpacity').value}%`;});
  function stop(message='Stopped. Completed actions are kept; Undo can remove them.') { token++;requestId=null;clearTimeout(deadline);setBusy(false);render();status(message); }
  $('drawStop').addEventListener('click',()=>stop());
  try { objects=documentObjects(JSON.parse(localStorage.getItem(KEY)||'[]')); }catch(_){objects=[];status('Saved artwork could not be restored. Open an exported artwork file to recover it.');}render();syncControls();
  $('drawingPrompt').addEventListener('submit',e=> {
    e.preventDefault();if(busy||!connected)return;const text=$('drawPrompt').value.trim();if(!text)return;finishGesture();
    requestId=uid();token++;setBusy(true);status('Grace is planning her drawing…');
    $('drawResponse').textContent='Grace is thinking about what to draw…';
    // Allow the local model time to produce a detailed plan, then replay it.
    deadline=setTimeout(()=>stop('Grace did not finish in time. Try a simpler request or check the AIVA chat log.'),720000);
    try { inputTopic.publish(new ROSLIB.Message({data:JSON.stringify({text,identifier:requestId,canvas:{objects,selected}})})); }catch(error){stop(error.message);}
  });
  if(!window.ROSLIB) { status('Could not load the ROS connection library. You can still draw by hand.');return; }
  const ros=new ROSLIB.Ros({url:`${location.protocol==='https:'?'wss:':'ws:'}//${location.host}/rosbridge`});
  inputTopic=new ROSLIB.Topic({ros,name:'/aiva/chat_input',messageType:'std_msgs/String'});
  const responses=new ROSLIB.Topic({ros,name:'/tms/text_response',messageType:'std_msgs/String'});
  ros.on('connection',()=>{connected=true;setBusy(busy);status('Connected. Ask Grace to draw, or use the tools yourself.');});
  ros.on('error',()=>{connected=false;stop('ROS connection unavailable. You can still draw by hand.');});
  ros.on('close',()=>{connected=false;stop('Disconnected. Reload this page after AIVA reconnects.');});
  responses.subscribe(message=> {
    let event;try{event=JSON.parse(message.data);}catch(_){return;}
    if(!requestId||event.identifier!==requestId)return;
    if(event.type==='canvas_start')status('Grace is planning her drawing…');
    else if(event.type==='canvas_reply')$('drawResponse').textContent=String(event.content||'').slice(0,1000);
    else if(event.type==='canvas_actions') {
      try {
        if(!Array.isArray(event.actions)||!event.actions.length||event.actions.length>100)throw new Error('Invalid drawing plan');
        const actions=event.actions.map(normalize),session=token;status('Grace is drawing…');
        replay=replay.then(async()=>{for(let i=0;i<actions.length;i++){if(session!==token)return;status(`Grace is drawing · ${i+1} / ${actions.length}`);await animate(actions[i],session);}}).catch(error=>{if(session===token)stop(error.message);});
      }catch(error){stop(error.message);}
    } else if(event.type==='canvas_done') {
      const session=token;replay.then(()=>{if(session===token&&requestId){clearTimeout(deadline);requestId=null;setBusy(false);status('Grace finished. You can edit the drawing or ask for changes.');}});
    } else if(event.type==='canvas_error') {
      const detail=String(event.content||'The drawing request failed.');
      $('drawResponse').textContent=`I couldn't finish that drawing: ${detail}`;
      stop(`Grace could not draw: ${detail}`);
    }
  });
  window.addEventListener('pagehide',()=>{token++;ros.close();},{once:true});
})();
