/* Run the shipped canvas script with DOM/ROS doubles; no GPU or browser install. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname,'../telepathic_management/telepathic_console/backend/static/drawing.js'),'utf8');
const PNG='data:image/png;base64,iVBORw0KGgo=';
const tick=()=>new Promise(setImmediate);

function studio() {
  const storage=new Map(), elements=new Map(), topics=new Map(), images=[], draws=[];
  const context=new Proxy({measureText:text=>({width:String(text).length*8}),
    drawImage:image=>draws.push(image.src||'canvas'),
    createLinearGradient:()=>({addColorStop(){}}),createRadialGradient:()=>({addColorStop(){}})},
    {get:(object,key)=>key in object?object[key]:()=>{}});
  class Element {
    constructor(id='') {this.id=id;this.value='';this.style={};this.handlers={};this.files=[];this.children=[];}
    addEventListener(type,handler){this.handlers[type]=handler;}
    async emit(type,event={}){return this.handlers[type]?.({target:this,preventDefault(){},...event});}
    replaceChildren(...children){this.children=children;}
    append(...children){this.children.push(...children);}
    setAttribute(){} getContext(){return context;} focus(){} click(){}
    toDataURL(){return PNG;}
  }
  const element=id=>{
    if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);
  };
  element('palettePreset').value='studio';element('drawOpacity').value='100';
  const document={getElementById:element,querySelectorAll:()=>[],
    querySelector:()=>new Element(),createElement:()=>new Element(),
    createDocumentFragment:()=>new Element(),addEventListener(){}};
  class Image {
    constructor(){images.push(this);}
    set src(src){this._src=src;} get src(){return this._src;}
  }
  class Ros {
    constructor(){this.handlers={};studio.ros=this;}
    on(type,handler){this.handlers[type]=handler;}
    close(){}
  }
  class Topic {
    constructor({name}){this.name=name;this.published=[];topics.set(name,this);}
    publish(message){this.published.push(message);}
    subscribe(handler){this.handler=handler;}
  }
  let nextId=0;
  const sandbox={document,Image,location:{protocol:'http:',host:'127.0.0.1'},
    window:{ROSLIB:true,matchMedia:()=>({matches:true}),addEventListener(){}},
    ROSLIB:{Ros,Topic,Message:class {constructor(value){Object.assign(this,value);}}},
    localStorage:{getItem:key=>storage.get(key)||null,setItem:(key,value)=>storage.set(key,value)},
    crypto:{randomUUID:()=>`id-${++nextId}`},performance,Event,Blob,URL,
    setTimeout:()=>1,clearTimeout(){},requestAnimationFrame:setImmediate};
  vm.runInNewContext(source,sandbox,{filename:'drawing.js'});
  studio.ros.handlers.connection();
  return {
    element,images,draws,
    artwork:()=>JSON.parse(storage.get('grace-canvas-v1')||'[]'),
    topic:name=>topics.get(name),
    respond:event=>topics.get('/tms/text_response').handler({data:JSON.stringify(event)}),
    async loadImages(){for(const image of images)if(!image.done){image.done=true;image.onload();}await tick();},
    async ask(){element('drawPrompt').value='Paint an apple';await element('drawingPrompt').emit('submit');
      return JSON.parse(topics.get('/aiva/chat_input').published.at(-1).data);},
  };
}

test('bitmap decoding finishes before overlays and vision snapshots, without resending PNGs',async()=>{
  const app=studio(),request=await app.ask(),identifier=request.identifier;
  app.respond({type:'canvas_image',identifier,id:'painting',src:PNG,prompt:'a red apple',x:0,y:0,width:1000,height:650});
  app.respond({type:'canvas_actions',identifier,actions:[{kind:'text',id:'caption',text:'Apple',x:40,y:40}]});
  app.respond({type:'canvas_snapshot_request',identifier});
  await tick();
  assert.equal(app.topic('/aiva/canvas_snapshot').published.length,0);
  await app.loadImages();
  assert.deepEqual(app.artwork().map(a=>a.id),['painting','caption']);
  assert.equal(app.topic('/aiva/canvas_snapshot').published.length,1);
  assert.ok(app.draws.includes(PNG));
  app.respond({type:'canvas_done',identifier});await tick();
  const followup=await app.ask();
  assert.equal(followup.canvas.objects[0].prompt,'a red apple');
  assert.equal('src' in followup.canvas.objects[0],false);
  assert.equal(followup.snapshot,PNG);
});

test('reopening artwork above the old 2 MB cap retains images and bounds their placement',async()=>{
  const app=studio();
  const object={kind:'image',id:'saved',src:PNG,x:990,y:640,width:512,height:384};
  app.element('importProject').files=[{size:3*1024*1024,text:async()=>JSON.stringify({version:1,objects:[object]})}];
  await app.element('importProject').emit('change');
  assert.equal(app.artwork()[0].x,488);assert.equal(app.artwork()[0].y,266);
  await app.loadImages();
  assert.equal(app.artwork()[0].src,PNG);
});

test('invalid image payload ends the request without adding a layer',async()=>{
  const app=studio(),request=await app.ask();
  app.respond({type:'canvas_image',identifier:request.identifier,id:'bad',src:'data:image/svg+xml,<svg/>',x:0,y:0,width:512,height:512});
  await tick();
  assert.deepEqual(app.artwork(),[]);
  assert.match(app.element('drawStatus').textContent,/Invalid painted image/);
  assert.equal(app.element('drawAsk').disabled,false);
});
