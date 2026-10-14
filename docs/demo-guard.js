/* Shared boundary for the static previews. Only same-origin assets are loaded. */
(()=>{
  'use strict';
  const nativeFetch=window.fetch.bind(window);
  const urlOf=input=>new URL(input instanceof Request?input.url:String(input),location.href);
  const json=(data,status=200)=>new Response(JSON.stringify(data),{status,headers:{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'}});
  const unavailable=()=>json({accepted:false,error:'在线演示：该操作需要在下载的本地中控中执行。 / Available in the local console only.'},403);
  function transport(route){
    window.fetch=async(input,options)=>{
      const url=urlOf(input),method=(options?.method||(input instanceof Request?input.method:'GET')).toUpperCase();
      if(url.origin!==location.origin)return unavailable();
      if(url.pathname.startsWith('/api/'))return await route(url,method,options)||unavailable();
      if(method!=='GET')return unavailable();
      return nativeFetch(input,options);
    };
  }
  class OfflineSocket extends EventTarget{
    static CONNECTING=0;static OPEN=1;static CLOSING=2;static CLOSED=3;
    constructor(){super();this.readyState=3;queueMicrotask(()=>this.onerror?.(new Event('error')));}
    close(){this.readyState=3;}
    send(){throw Error('Online demo: device connections are disabled.');}
  }
  window.WebSocket=OfflineSocket;
  window.RTCPeerConnection=class{constructor(){throw Error('Online demo: camera connections are disabled.');}};
  navigator.sendBeacon=()=>false;
  function download(content,name,type='application/json'){
    const url=URL.createObjectURL(new Blob([content],{type})),link=document.createElement('a');
    link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  function lock(selector){
    const apply=()=>document.querySelectorAll(selector).forEach(el=>{if(!el.disabled)el.disabled=true;el.title='在线演示仅供查看 / Online preview only';});
    const observer=new MutationObserver(apply);observer.observe(document.body,{subtree:true,childList:true,attributes:true,attributeFilter:['disabled']});apply();
    document.addEventListener('click',event=>{if(event.target.closest(selector)){event.preventDefault();event.stopImmediatePropagation();}},true);
  }
  function banner(before,text,href,label){
    const el=document.createElement('div');el.id='online-demo-notice';el.className='banner';el.setAttribute('role','status');
    el.style.cssText='margin:12px 0;border:1px solid #557548;background:#253323;color:#d8fbaa;line-height:1.7;font-size:13px';
    el.append(document.createTextNode(text+' '));const link=document.createElement('a');link.href=href;link.textContent=label;link.style.color='#c1f577';el.append(link);
    document.querySelector(before).before(el);
  }
  window.MicroduckDemo={json,transport,asset:(path)=>nativeFetch(new URL(path,document.baseURI)).then(r=>{if(!r.ok)throw Error('Demo asset: '+path);return r.json();}),lock,banner,download};
})();
