/* Retained drawing objects allow both Grace and the user to edit the same artwork. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const canvas = $('drawingCanvas'), ctx = canvas.getContext('2d');
  const WIDTH = 1000, HEIGHT = 650, KEY = 'grace-canvas-v1';
  const kinds = new Set(['path','erase','line','rectangle','ellipse','text','letter','select','move','delete','clear','undo','redo']);
  let objects = [], undo = [], redo = [], selected = null, gesture = null;
  let busy = false, connected = false, token = 0, requestId = null, deadline = null;
  let inputTopic, replay = Promise.resolve();
  const clone = value => JSON.parse(JSON.stringify(value));
  const status = text => { $('drawStatus').textContent = text; };
  const uid = () => globalThis.crypto?.randomUUID?.() || `shape-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  function save() { try { localStorage.setItem(KEY, JSON.stringify(objects)); } catch (_) { status('Browser storage is full; use Save PNG to keep your artwork.'); } }
  function checkpoint() { undo.push(clone(objects)); if (undo.length > 50) undo.shift(); redo = []; }
  function style(a) { ctx.strokeStyle = a.color || '#183b65'; ctx.fillStyle = a.color || '#183b65'; ctx.lineWidth = Math.max(1, a.size || 5); ctx.lineCap = ctx.lineJoin = 'round'; }
  // Lay out the complete letter before replay so wrapping stays stable as text appears.
  function letterLayout(a) {
    const x=a.x??270, y=a.y??20, width=a.width??460, height=a.height??610;
    const margin=a.margin??40, size=Math.max(8,a.size??16), step=size*(a.line_height??1.5);
    const available=width-2*margin, lines=[];
    if(available<size || height-2*margin<step || x+width>WIDTH || y+height>HEIGHT) throw new Error('Letter page does not fit. Adjust its size, position, or margins.');
    ctx.save(); ctx.font=`${size}px Georgia, serif`;
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
  function paint(a, progress = 1) {
    ctx.save(); style(a);
    if (a.kind === 'erase') ctx.globalCompositeOperation = 'destination-out';
    if (['path','erase','line'].includes(a.kind)) {
      const points = a.points || []; if (!points.length) { ctx.restore(); return; }
      const travel = Math.max(0, (points.length - 1) * progress), whole = Math.floor(travel);
      ctx.beginPath(); ctx.moveTo(...points[0]);
      for (let i = 1; i <= whole; i++) ctx.lineTo(...points[i]);
      if (whole < points.length - 1) { const f=travel-whole;ctx.lineTo(points[whole][0]+(points[whole+1][0]-points[whole][0])*f,points[whole][1]+(points[whole+1][1]-points[whole][1])*f); }
      if (points.length === 1) ctx.lineTo(points[0][0]+.01, points[0][1]);
      ctx.stroke();
    } else if (a.kind === 'letter') {
      const page=letterLayout(a), {x,y,width,height,margin,size,step,lines}=page;
      ctx.fillStyle='#ffffff';ctx.fillRect(x,y,width,height);
      ctx.strokeStyle='#b6c4d3';ctx.lineWidth=1;ctx.strokeRect(x,y,width,height);
      ctx.font=`${size}px Georgia, serif`;ctx.textBaseline='top';ctx.textAlign=a.align||'left';ctx.fillStyle=a.color||'#183b65';
      const left=x+margin, right=x+width-margin, anchor=a.align==='center'?(left+right)/2:a.align==='right'?right:left;
      let remaining=Math.ceil(lines.reduce((n,line)=>n+Array.from(line).length,0)*progress);
      lines.forEach((line,i)=>{const chars=Array.from(line);ctx.fillText(chars.slice(0,Math.max(0,remaining)).join(''),anchor,y+margin+i*step);remaining-=chars.length;});
    } else if (a.kind === 'text') {
      ctx.font = `${Math.max(8,a.size || 24)}px sans-serif`; ctx.textBaseline = 'top';
      const text = (a.text || '').slice(0, Math.ceil((a.text || '').length * progress));
      text.split('\n').forEach((line,i) => ctx.fillText(line, a.x || 0, (a.y || 0) + i * (a.size || 24) * 1.2));
    } else {
      ctx.beginPath();
      if (a.kind === 'ellipse') ctx.ellipse((a.x||0)+(a.width||0)/2,(a.y||0)+(a.height||0)/2,Math.abs(a.width||0)/2,Math.abs(a.height||0)/2,-Math.PI/2,0,Math.PI*2*progress);
      else if (a.kind === 'rectangle') {
        const x=a.x||0,y=a.y||0,w=a.width||0,h=a.height||0, pts=[[x,y],[x+w,y],[x+w,y+h],[x,y+h],[x,y]];
        ctx.moveTo(x,y); const edges=progress*4;
        for(let i=0;i<4;i++) { if(edges>=i+1) ctx.lineTo(...pts[i+1]); else if(edges>i) ctx.lineTo(pts[i][0]+(pts[i+1][0]-pts[i][0])*(edges-i),pts[i][1]+(pts[i+1][1]-pts[i][1])*(edges-i)); }
      }
      if (a.fill && progress === 1) ctx.fill(); ctx.stroke();
    }
    ctx.restore();
  }
  function bounds(a) {
    if(a.kind==='letter') { const p=letterLayout(a);return [p.x,p.y,p.width,p.height]; }
    if (a.points) { const xs=a.points.map(p=>p[0]), ys=a.points.map(p=>p[1]); return [Math.min(...xs),Math.min(...ys),Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys)]; }
    if(a.kind==='text') { ctx.font=`${a.size||24}px sans-serif`; const lines=(a.text||'').split('\n'); return [a.x||0,a.y||0,Math.max(...lines.map(l=>ctx.measureText(l).width)),lines.length*(a.size||24)*1.2]; }
    return [a.x||0,a.y||0,a.width||0,a.height||0];
  }
  function render(preview = null, progress = 1, selection = true) {
    ctx.clearRect(0,0,WIDTH,HEIGHT); objects.forEach(a=>paint(a));
    if (preview) paint(preview,progress);
    const active=objects.find(a=>a.id===selected);
    if(selection && active) { ctx.save(); ctx.strokeStyle='#008cc3';ctx.lineWidth=1.5;ctx.setLineDash([5,4]);const [x,y,w,h]=bounds(active);ctx.strokeRect(x-4,y-4,w+8,h+8);ctx.restore(); }
    $('drawLetter').textContent=active?.kind==='letter'?'Update letter page':'Add letter page';
  }
  function editLetter() {
    const a=objects.find(o=>o.id===selected);if(a?.kind!=='letter')return;
    $('drawText').value=a.text||'';$('drawSize').value=a.size??16;$('drawColor').value=a.color||'#183b65';
    $('letterAlign').value=a.align||'left';$('letterMargin').value=a.margin??40;$('letterSpacing').value=String(a.line_height??1.5);
  }
  function move(a,dx,dy) {
    if(a.points) a.points=a.points.map(([x,y])=>[x+dx,y+dy]);
    else if(a.kind==='letter') { a.x=Math.max(0,Math.min(WIDTH-a.width,a.x+dx));a.y=Math.max(0,Math.min(HEIGHT-a.height,a.y+dy)); }
    else { a.x=(a.x||0)+dx;a.y=(a.y||0)+dy; }
  }
  function action(a) {
    if(a.kind==='select') { if(!objects.some(o=>o.id===a.id)) throw new Error(`Object not found: ${a.id}`); selected=a.id;editLetter(); }
    else if(a.kind==='undo') { if(undo.length) { redo.push(clone(objects));objects=undo.pop();selected=null; } }
    else if(a.kind==='redo') { if(redo.length) { undo.push(clone(objects));objects=redo.pop();selected=null; } }
    else if(a.kind==='clear') { checkpoint();objects=[];selected=null; }
    else if(['move','delete'].includes(a.kind)) {
      const id=a.id||selected, target=objects.find(o=>o.id===id);if(!target) throw new Error('Select an object first.');
      checkpoint();if(a.kind==='delete') { objects=objects.filter(o=>o.id!==id);selected=null; } else move(target,a.dx||0,a.dy||0);
    } else {
      if(a.kind==='letter') letterLayout(a);
      if(objects.length>=100) throw new Error('Canvas has 100 objects. Delete some before adding more.');
      if(a.id && objects.some(o=>o.id===a.id)) throw new Error(`Object ID already exists: ${a.id}`);
      checkpoint(); objects.push({...a,id:a.id||uid()}); selected=null;
    }
    render();save();
  }
  function setBusy(value) {
    busy=value; $('drawAsk').disabled=value||!connected; $('drawStop').disabled=!value;
    document.querySelectorAll('.tools input,.tools select,.tools textarea,.tools button,.drawing-actions button').forEach(el=> { if(el.id!=='drawStop' && el.id!=='drawExport') el.disabled=value; });
  }
  async function animate(a, session) {
    if(['select','move','delete','clear','undo','redo'].includes(a.kind)) { action(a); return; }
    if(a.kind==='letter') letterLayout(a);
    const duration=window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 0 : (['text','letter'].includes(a.kind) ? Math.min(6000,(a.text||'').length*40) : 600);
    const start=performance.now();
    while(duration && performance.now()-start<duration) { if(session!==token) { render();return; } render(a,(performance.now()-start)/duration,false);await new Promise(requestAnimationFrame); }
    if(session===token) action(a);
  }
  function normalize(a) {
    if(!a || !kinds.has(a.kind)) throw new Error('Unknown drawing action');
    const out={kind:a.kind};
    for(const key of ['x','y','width','height','dx','dy','size','margin','line_height']) if(key in a) { if(typeof a[key]!=='number'||!Number.isFinite(a[key])) throw new Error('Invalid drawing coordinates');const max={x:WIDTH,y:HEIGHT,width:WIDTH,height:HEIGHT,dx:WIDTH,dy:HEIGHT,size:120,margin:100,line_height:2}[key];out[key]=Math.max(key==='dx'||key==='dy'?-max:key==='line_height'?1:0,Math.min(max,a[key])); }
    if('align' in a) { if(!['left','center','right'].includes(a.align)) throw new Error('Invalid letter alignment');out.align=a.align; }
    for(const key of ['text','id']) if(key in a) { if(typeof a[key]!=='string'||a[key].length>(key==='text'?2000:64)) throw new Error('Invalid drawing text');out[key]=a[key]; }
    if(a.color) { if(!/^#[0-9a-f]{6}$/i.test(a.color)) throw new Error('Invalid color');out.color=a.color; }
    out.fill=a.fill===true;
    if(['line','path','erase'].includes(a.kind)) { if(!Array.isArray(a.points)||!a.points.length||a.points.length>2000) throw new Error('Invalid drawing path');out.points=a.points.map(p=>{if(!Array.isArray(p)||p.length!==2||p.some(v=>typeof v!=='number'||!Number.isFinite(v))) throw new Error('Invalid path point');return [Math.max(0,Math.min(WIDTH,p[0])),Math.max(0,Math.min(HEIGHT,p[1]))];}); }
    if(out.kind==='letter') {
      Object.assign(out,{x:out.x??270,y:out.y??20,width:out.width??460,height:out.height??610,size:out.size??16});
      letterLayout(out);
    }
    return out;
  }
  function point(e) { const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(WIDTH,(e.clientX-r.left)*WIDTH/r.width)),Math.max(0,Math.min(HEIGHT,(e.clientY-r.top)*HEIGHT/r.height))]; }
  canvas.addEventListener('pointerdown',e=> {
    if(busy || e.button!==0) return; e.preventDefault();canvas.focus();canvas.setPointerCapture(e.pointerId);
    const p=point(e), kind=$('drawTool').value;
    if(kind==='select') { selected=null;for(let i=objects.length-1;i>=0;i--) { const a=objects[i];if(a.kind==='erase') continue; const [x,y,w,h]=bounds(a);if(p[0]>=x-8&&p[0]<=x+w+8&&p[1]>=y-8&&p[1]<=y+h+8){selected=a.id;break;} } editLetter();gesture={kind,start:p,original:clone(objects)};render();return; }
    const a={kind,color:$('drawColor').value,size:Math.max(1,Math.min(120,Number($('drawSize').value)||5)),fill:$('drawFill').checked};
    if(kind==='text') { a.text=$('drawText').value; a.x=p[0];a.y=p[1];if(a.text) action(a);return; }
    a.points=[p];gesture={kind,start:p,object:a};
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
    if(g.kind==='select') { if(cancel)objects=g.original;else if(selected){undo.push(g.original);redo=[];save();} }
    else if(!cancel) { try{action(normalize(g.object));}catch(e){status(e.message);} }render();
  }
  $('drawTool').addEventListener('change',()=>{if($('drawTool').value==='text')$('drawSize').value=24;});
  $('drawLetter').addEventListener('click',()=> {
    if(busy)return;
    const current=objects.find(a=>a.id===selected&&a.kind==='letter');
    try {
      const a=normalize({...current,kind:'letter',text:$('drawText').value,color:$('drawColor').value,
        size:current?Number($('drawSize').value):16,align:$('letterAlign').value,
        margin:Number($('letterMargin').value),line_height:Number($('letterSpacing').value)});
      if(!a.text.trim())throw new Error('Enter your letter in the Text field first.');
      if(current) { checkpoint();objects[objects.indexOf(current)]=a;render();save(); }
      else action(a);
      status('Letter page saved. Use Select / move to edit it, or Save PNG to export it.');
    }catch(error){status(error.message);}
  });
  canvas.addEventListener('pointerup',()=>finishGesture());canvas.addEventListener('pointercancel',()=>finishGesture(true));
  for(const [id,kind] of [['drawUndo','undo'],['drawRedo','redo'],['drawDelete','delete'],['drawClear','clear']]) $(id).addEventListener('click',()=>{if(!busy)try{action({kind});}catch(e){status(e.message);}});
  canvas.addEventListener('keydown',e=>{if(!busy&&(e.key==='Delete'||e.key==='Backspace')){e.preventDefault();try{action({kind:'delete'});}catch(error){status(error.message);}}});
  $('drawExport').addEventListener('click',()=>{render(null,1,false);const output=document.createElement('canvas');output.width=WIDTH;output.height=HEIGHT;const exportCtx=output.getContext('2d');exportCtx.fillStyle='white';exportCtx.fillRect(0,0,WIDTH,HEIGHT);exportCtx.drawImage(canvas,0,0);const link=document.createElement('a');link.download='grace-drawing.png';link.href=output.toDataURL('image/png');link.click();render();});
  function stop(message='Stopped. Completed actions are kept; Undo can remove them.') { token++;requestId=null;clearTimeout(deadline);setBusy(false);render();status(message); }
  $('drawStop').addEventListener('click',()=>stop());
  try { const saved=JSON.parse(localStorage.getItem(KEY)||'[]');if(Array.isArray(saved)&&saved.length<=100) objects=saved.map(normalize); }catch(_){objects=[];}render();
  $('drawingPrompt').addEventListener('submit',e=> {
    e.preventDefault();if(busy||!connected)return;const text=$('drawPrompt').value.trim();if(!text)return;finishGesture();
    requestId=uid();token++;setBusy(true);status('Grace is planning her drawing…');
    $('drawResponse').textContent='Grace is thinking about what to draw…';
    deadline=setTimeout(()=>stop('Grace did not respond in time. You can try again.'),150000);
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
        replay=replay.then(async()=>{for(let i=0;i<actions.length;i++){if(session!==token)return;status(`Grace is drawing · ${i+1} / ${actions.length}`);await animate(actions[i],session);}}).catch(error=>stop(error.message));
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
