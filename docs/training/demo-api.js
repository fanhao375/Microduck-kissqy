/* Training Studio R1.5.27 — isolated, read-only GitHub Pages preview. */
(()=>{"use strict";
const task="Mjlab-VelStand-Rough-Backlash-MicroDuck";
const now=()=>Date.now()/1000;
const points=Array.from({length:43},(_,i)=>{
 const iteration=Math.round(i*9999/42),progress=i/42,noise=Math.sin(i*1.4)*5;
 return {iteration,total:10000,reward:25+100*(1-Math.exp(-progress*3.5))+noise,
   fps:58000+Math.sin(i*.48)*6000,episode_length:360+progress*190,
   value_loss:.41*Math.exp(-progress*2)+.08,policy_loss:-.006*(1-progress),
   rewards:{track_linear_velocity:14+progress*24,track_angular_velocity:3+progress*5,
     upright:6+progress*10,expert_bc:.8-progress*.32},expert_bc:.85-progress*.3,
   expert_bc_anchor_frac:.62,expert_bc_fallen_frac:.27};
});
const job={
 id:"dem0abcd00000001",op:"train",status:"completed",created:now()-86400,ended:now()-43000,
 message:"示例训练已完成 · 页面所有曲线与记录均为 Demo 数据",profile:{mode:"wsl",distro:"Ubuntu",repo:"~/microduck-training-studio/engines/official_0151"},
 request:{task,label:"[DEMO] 行走与倒地起身",num_envs:4096,iterations:10000,
   training_action_scale:1,training_firmware_p:5,
   learning_rate:0.0003,seed:42,actor_dims:[256,128,64],action_filter:{enabled:false,head_alpha:1,legs_alpha:1}},
 effective_config:{num_envs:4096,iterations:10000,training_action_scale:1,
    training_firmware_p:5,action_filter:{enabled:false,head_alpha:1,legs_alpha:1}},
 resolved_task:task,target_compatible:true,resume_compatible:true,
 progress:{done:10000,total:10000,percent:100,elapsed_seconds:24800,eta_seconds:0},
 metrics:points,latest_metric:points.at(-1),logs:[
   "[DEMO] 独立训练中控 R1.5.27","[DEMO] GPU: RTX 4080 SUPER 16GB",
   "[DEMO] 4096 environments · 10000 iterations",
   "[DEMO] Export ready (sample only; no actual ONNX)"
 ],checkpoints:[{path:"/demo/model_9999.pt",name:"model_9999.pt",iteration:9999,modified:now()-43000}],
 compatibility_label:"Demo · HD1910",message_long:""
};
const profile=job.profile;
function currentState(){
 return {profile,computer_verified:true,
  environment:{ready:true,status:"ready",reused:true,engine:"official_0151",
   message:"[DEMO] 训练环境为模拟状态，不运行 CUDA / WSL / 训练程序",
   revision:"8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec",
   gpu:{gpus:[{name:"RTX 4080 SUPER · DEMO",memory_gb:16}]},
   configs:{},tasks:[task]},
  jobs:[job],
  resources:{received:now(),devices:[{name:"RTX 4080 SUPER · DEMO",utilization:81,
    used_mb:11260,total_mb:16384,temperature:65}]},
  presets:[],training_queue:{enabled:false,entries:[{
    id:"demo-queue",status:"completed",job_id:job.id,request:job.request,
    progress:job.progress,recipe_sha256:"demo0000",message:"示例任务已完成"}],
    failure_policy:"pause",message:"[DEMO] 训练队列为静态示例，不能启动新任务"},
  };
}
const config={token:"DEMO-READONLY",version:"Studio R1.5.27 · ONLINE DEMO",
 platform:"linux",release:"R1.5.27",remote_workspace:""};
const D=window.MicroduckDemo;
const dataReady=Promise.all(['baseline_official0151','hardware','servo_references','pins'].map(name=>D.asset('./data/'+name+'.json'))).then(([baseline,hardware,servo_references,pins])=>{
  const calibration=structuredClone(hardware.current_calibration);calibration.source='公开样机标定示例 · DEMO';
  return {recipe:{schema:'microduck-training-recipe/v1',version:1,name:'官方骨架 + HD1910 · 演示配置',baseline:'official_0151',purpose:'sim2real',training_action_scale:1,robot:{servo_mass_model:'hd1910_820g_v1'},overrides:[],task_overrides:{},calibration},
    zero_connection:{target:''},pins,baseline,hardware,servo_references,
    servo_mass_profile:{key:'hd1910_23g',reference_g:18,replacement_g:23,body_counts:{trunk_base:2,yaw2roll:1,upper_leg_left:2,leg:1,neck:2,yaw_roll_motion:1,jaw_soft:2,bearing_roll:1,upper_leg_right:2,leg_2:1}},
    mass_closure_profile:{key:'hd1910_820g_v1',target_g:820,body_additions_g:{jaw_soft:3,trunk_base:4.75682}},
    bundled_calibration_matches:true,target_runtime:'0.15.1',recipe_hash:'DEMO',checks:[],frozen:null,archives:[],readiness:{items:[]}};
});
const references=new Map();
function taskReference(id){
  if(!/^Mjlab-[A-Za-z0-9-]+$/.test(id))return Promise.reject(Error('Unknown demo task'));
  if(!references.has(id))references.set(id,D.asset('./data/task-configs/'+id+'.json'));
  return references.get(id);
}
D.transport(async(url,method,options)=>{
  const path=url.pathname;
  if(method==='GET'){
    if(path==='/api/config')return D.json(config);
    if(path==='/api/state')return D.json(currentState());
    if(path==='/api/task-reference'){
      try{return D.json(await taskReference(url.searchParams.get('task')));}catch{return D.json({error:'该任务没有演示配置。'},404);}
    }
    if(path==='/api/studio')return D.json(await dataReady);
    if(path==='/api/remote/state')return D.json({target:'local',status:'ready',message:'[DEMO] 仅展示服务器设置，不连接 SSH',selected:'local',connected:false,wsl_test_supported:false,connection:{host:'',port:22,user:'',auth:'password'}});
    if(path==='/api/log')return new Response(job.logs.join('\n'),{headers:{'Content-Type':'text/plain; charset=utf-8'}});
  }
  // Configuration edits live in this tab only. No settings are uploaded or persisted.
  if(method==='POST'&&path==='/api/recipe'){
    const data=await dataReady,payload=JSON.parse(options?.body||'{}');data.recipe=payload.recipe||data.recipe;return D.json({recipe:data.recipe,demo:true});
  }
});
document.addEventListener('DOMContentLoaded',()=>{
  document.body.classList.add('pages-training-demo');
  D.banner('.train-top','ONLINE DEMO · Training Studio R1.5.27 · 新版界面；奖励曲线、GPU 与训练记录均为示例，配置只保留在当前页面，不能实际训练或连接 SSH。','../','← 返回机器人中控演示');
  D.lock('#train-start,#train-stop,#queue-add,#queue-clear,[data-queue-action],#environment-setup,#probe,#preview-start,#play-start,#recovery-eval-start,#export-model,#calibration-zero,#calibration-file,#calibration-bundled,#ssh-connect,#ssh-deploy,#ssh-disconnect,#ssh-save,#ssh-restore,#ssh-trust-host,#ssh-password,#wsl-test,#server-model-upload,#delete-model,#training-target,[data-delete-model],[data-download-model]');
  document.addEventListener('submit',e=>{e.preventDefault();e.stopImmediatePropagation();},true);
  document.addEventListener('click',e=>{
    const link=e.target.closest('a[href^="/api/"]');if(!link)return;e.preventDefault();e.stopImmediatePropagation();
    if(link.id==='download-run')D.download(JSON.stringify({demo:true,version:'R1.5.27',job},null,2),'microduck-training-demo-record.json');
  },true);
});
window.__trainingDemo={state:currentState,config,studio:()=>dataReady};
})();
