/* Public offline preview adapter: no outgoing API or hardware commands. */
(()=>{
"use strict";

const ids=[20,21,22,23,24,10,11,12,13,14,30,31,32,33,34];
const names=["左髋Yaw","左髋Roll","左髋Pitch","左膝","左踝","右髋Yaw","右髋Roll","右髋Pitch","右膝","右踝","颈Pitch","头Pitch","头Yaw","头Roll","嘴"];
const homeDeg=[0,-5,-26.24,-0.28,25.95,0,5,26.24,0.28,-25.95,20,20,0,0,0];
const config={
 version:"1.0.167",baseline:"1fa84386f07884e27866411bc1ba166977bced95",mode:"service",
 service_enabled:true,control_enabled:false,connection_ui:false,raw_telemetry:false,
 control_token:"",ssh_target:"",camera_host:"duck-demo.invalid",camera_port:8443,
 hz:50,ids,labels:names,home:homeDeg.map(x=>x*Math.PI/180)
};
const start=Date.now();
const channel=()=>({status:"live",age_ms:20,error:null});
const models=[{"id": "1cbf8730eadd75a5704332c9f3c10f60", "name": "行走＋倒地联合 9600 轮", "slot": "walk", "task": "Mjlab-VelStand-Rough-Backlash-MicroDuck", "sha256": "1cbf8730eadd75a5704332c9f3c10f60bfa9412135b907831c9dbd731ce1795f", "available": true, "active": true, "demo": true, "runtime_execution_profile": {"schema": "microduck-runtime-execution/v1", "model_sha256": "1cbf8730eadd75a5704332c9f3c10f60bfa9412135b907831c9dbd731ce1795f", "mode": "uniform", "action_scale": 1, "head_lowpass": 0.5, "legs_lowpass": 0.7}}, {"id": "1c94934c6421ec72ed48079cea749668", "name": "坐下／站起 10000 轮", "slot": "sitstand", "task": "Mjlab-SitStand-Flat-MicroDuck", "sha256": "1c94934c6421ec72ed48079cea74966868a7a59f7d732e0207d0f7bdaeed3c50", "available": true, "active": true, "demo": true}, {"id": "b14e3a8b56ddbb8d2d831e727774a168", "name": "低头捡取 10000 轮", "slot": "ground_pick", "task": "Mjlab-GroundPick-Flat-Backlash-MicroDuck", "sha256": "b14e3a8b56ddbb8d2d831e727774a168c8f5b00194e3e431ae7593a840498c69", "available": true, "active": true, "demo": true}, {"id": "59e15d7b04d0c40e9b11b8a70bdecae6", "name": "前滚翻 10000 轮", "slot": "roulade", "task": "Mjlab-Roulade-Flat-MicroDuck", "sha256": "59e15d7b04d0c40e9b11b8a70bdecae6da87a5e096efc204fb58eea304fcbbac", "available": true, "active": true, "demo": true}, {"id": "67ad09d82f4865d222a7ae06f250cf9f", "name": "右脚踢球 10000 轮", "slot": "kick_right", "task": "Mjlab-BallKick-Flat-MicroDuck", "sha256": "67ad09d82f4865d222a7ae06f250cf9f77f08c27e4f019ee3f7e431a764419fd", "available": true, "active": true, "demo": true}, {"id": "ee0376c927044cdece056ba1d3e6eac7", "name": "左脚踢球 10000 轮", "slot": "kick_left", "task": "Mjlab-BallKickLeft-Flat-Backlash-MicroDuck", "sha256": "ee0376c927044cdece056ba1d3e6eac77b3e02ee3f701f1feb437c183ed39a6d", "available": true, "active": true, "demo": true}];
const loadedModels=Object.fromEntries(models.map(row=>[row.slot,row]));
function snapshot(){
  const seconds=(Date.now()-start)/1000,now=Date.now()/1000;
  const joints=homeDeg.map((v,i)=>(v+2.2*Math.sin(seconds*1.25+i*.61))*Math.PI/180);
  const targets=homeDeg.map(v=>v*Math.PI/180);
  const servos=ids.map((id,i)=>{
    const angle=joints[i]*180/Math.PI,goal=homeDeg[i];
    return {id,name:names[i],label:names[i],status:"live",source:"Fake Robot",
      position_raw:Math.round(2048+angle*4096/360),goal_position_raw:Math.round(2048+goal*4096/360),
      angle_deg:angle,target_deg:goal,error_deg:goal-angle,calibrated:true,
      voltage_v:7.28+.07*Math.sin(seconds+i*.4),temperature_c:36+i%5+Math.sin(seconds*.6+i),
      current_ma:130,current_raw:86,speed_raw:3,load_raw:28,velocity_rpm:.4,
      torque_enabled:true,moving:false,model_number:1910,
      p_gain:5,i_gain:0,d_gain:20,hardware_error:0,
      calibration:{calibrated:true,zero_raw:2048,direction:i%3===0?-1:1,min_rad:-1.57,max_rad:1.57}
    };
  });
  const motorTemps=servos.map(x=>x.temperature_c);
  const roll=1.4*Math.sin(seconds*.33),pitch=1.2*Math.sin(seconds*.27);
  const bus={mode:"motion",phase:"control",build:"1fa8438-feetech-ft6-control.43",
    management_capabilities:{supported:true,management:true,motion:true,webpad:false,live_pad_settings:false,coast:true},
    torque_state:"on",policy_enabled:false,homed:true,scale_use:"walking",
    action_scale:1,head_lowpass:.5,legs_lowpass:.7,voltage_adapt:true,
    nominal_voltage:7.3,voltage_scale_mult:1,active_action_scale:1,
    loaded_models:loadedModels,
    home_start_guard:{ready:true,max_joint_error_deg:2.1,tilt_deg:1.2,reason:"Demo"},
    total_cycles:Math.floor(seconds*50),cycle:Math.floor(seconds*50),achieved_hz:50+.2*Math.sin(seconds),achieved_hz:50+.2*Math.sin(seconds),
    total_communication_failures:0,session:"demo",target_hz:50,
    native:{imu_ready:true,torque_state_confirmed:true,command_clamps_total:0,
      sync:{address:56,length:15,request_sent:true,missing_ids:[],elapsed_us:2400}},
    diagnostics:{missing_by_id:{}},error:""
  };
  const health={healthy:true,degraded:false,reason:"",
    control_loop:{target_hz:50,hz:50+.2*Math.sin(seconds),achieved_hz:50+.2*Math.sin(seconds),last_tick_age_ms:10,missed:0},
    battery:{volts:7.25},motors:{max_c:Math.max(...motorTemps),mean_c:motorTemps.reduce((a,b)=>a+b,0)/15,hottest:"ID 20"},
    bus:{consecutive_errors:0,startup_failures:0},imu:{ready:true,stale_blocks:0,consecutive_stale_blocks:0}
  };
  const system={hostname:"duck-demo",cpu_percent:15+Math.sin(seconds)*4,
    cpu_temp_c:62+Math.sin(seconds*.2)*2,memory_total:2147483648,
    memory_available:1258291200,uptime_s:5400+seconds,
    identities:{robotd:{revision:"1fa84386f07884e27866411bc1ba166977bced95"}},
    camera:{fps:30,targetFps:30,width:1280,height:720,format:"NV12",dropped:0,consumers:0},
    camera_age_ms:20
  };
  return {schema:"microduck-console/v1",mode:"service",version:"1.0.167",baseline:config.baseline,at:now,uptime:seconds,
    read_only:true,service_sample_fresh:true,pose_calibrated:true,service_compatible:true,
    service_operation:false,connection_paused:false,extended:false,
    channels:{state:channel(),health:channel(),system:channel(),tof:channel(),
      bus:channel(),calibration:channel(),capabilities:channel(),startup:channel()},
    startup:{steps:Object.fromEntries([1,2,3,4].map(n=>[n,{status:"success",detail:"DEMO"}]))},
    state:{joints,targets,odom:{yaw:.1*Math.sin(seconds*.1),position:[.02*Math.sin(seconds*.1),0,0]},
      move:{requested:[0,0,0],applied:[0,0,0],limited_by:[]},
      safety:{fallen:false,gain:5},loop:{missed:0,hz:50+.2*Math.sin(seconds)},policy:"Demo · 联合行走／起身"},
    bus,health,system,servos,
    capabilities:{walk:loadedModels.walk.name,sitstand:loadedModels.sitstand.name,ground_pick:loadedModels.ground_pick.name,skills:["roulade","kick_right","kick_left"]},
    control:{connected:false,audio:{volume:72},webpad:{available:false,real_connected:false,
      settings:{mouth_percent:50,head_rad:1},error:"在线只读展示"}},
    imu:{status:"live",ready:true,mount_verified:true,source:"Fake IMU",
      gravity:[Math.sin(roll*Math.PI/180),Math.sin(pitch*Math.PI/180),-.999],
      gyro:[.02*Math.sin(seconds),.01,0],quat:[1,0,0,0],roll_deg:roll,pitch_deg:pitch},
    tof:{rows:8,cols:8,seq:Math.floor(seconds*8),distance_mm:Array.from({length:64},(_,i)=>350+30*(i%8)+Math.round(12*Math.sin(seconds+i))),status:Array(64).fill(5)},
    calibration:{calibration_path:"Fake calibration · DEMO",calibration:{imu_mount_quat:[1,0,0,0],imu_mount_verified:true}},
    calibrated_count:15,saved_calibrated_count:15,service_control:{busy:false,stopping:false,job:null,sudo_password_saved:false},
    connection:{active:false},events:[]};
}
const D=window.MicroduckDemo;
const rollerNames={swizzle:'滑轮对称滑',crouch:'滑轮蹲下',spin:'滑轮原地转',standup:'滑轮倒地起身'};
const catalog={models,slots:models.map(model=>({slot:model.slot,model})),
  roller_models:Object.entries(rollerNames).map(([id,name])=>({id,name,available:true,demo:true,action_scale:1,head_lowpass:1,legs_lowpass:1})),
  management_actions:[],action_scale:1,default_action_scale:.7,default_joint_action_scale:1};
const recordings=[];
D.transport((url,method)=>{
  const path=url.pathname;
  if(method==='GET'){
    if(path==='/api/config')return D.json(config);
    if(path==='/api/snapshot'){
      const frame=snapshot();recordings.push(frame);if(recordings.length>600)recordings.shift();return D.json(frame);
    }
    if(path==='/api/system-log')return D.json({entries:[{seq:1,at:start/1000,message:'[DEMO] R17 V167 / control.43 · 所有机器人读数为模拟数据，不连接实机'}],cursor:1});
  }
  // These POST routes only read fixture metadata. All command routes are blocked.
  if(method==='POST'&&path==='/api/service/models')return D.json(catalog);
  if(method==='POST'&&path==='/api/service/connection')return D.json({management_revision:'R17',process_matches:true,running:true,actual_port:'/dev/ttyS2 · DEMO',pid:1042,actual_mode:'motion',motion_ready:true});
  if(method==='POST'&&path==='/api/service/firmware')return D.json({verified:false,error:'在线演示不读取或安装固件；正式版内含 control.43。'});
});
document.addEventListener('DOMContentLoaded',()=>{
  document.body.classList.add('pages-demo');
  D.banner('main .page-heading','ONLINE DEMO · R17 V167 / control.43 · 新版界面与官方 3D 模型；设备读数为模拟值，不能连接或控制实机。','./training/','打开训练中控演示 →');
  D.lock('#ssh-connect,#ssh-disconnect,#ssh-target,#service-sudo-password,#service-sudo-forget,#camera-connect,#camera-disconnect,#camera-save,#service-mode-calibrate,#service-mode-motion,#service-minus,#service-plus,#service-move,#service-capture,#service-center,#service-verify-imu,#service-init,#service-quick-stop,#service-reboot,#service-shutdown,#service-resume,#management-reconnect,#management-scale-apply,#firmware-choose,#firmware-file,#firmware-flash,#inspection-start,#inspection-stop,#gamepad-repair-start');
  document.getElementById('camera-message').textContent='在线演示不连接摄像头';
  document.addEventListener('click',event=>{
    const link=event.target.closest('a[href^="/api/"]');if(!link)return;
    event.preventDefault();event.stopImmediatePropagation();
    if(link.pathname==='/api/servos.csv'){
      const rows=snapshot().servos.map(r=>[r.id,r.name,r.angle_deg,r.voltage_v,r.temperature_c]);
      D.download('id,name,angle_deg,voltage_v,temperature_c\n'+rows.map(r=>r.join(',')).join('\n'),'microduck-demo-servos.csv','text/csv');
    }else if(link.pathname==='/api/system-log/download')D.download('[DEMO] R17 V167 / control.43 — simulated robot data only.','microduck-demo-log.txt','text/plain');
    else if(link.pathname==='/api/recording')downloadRecording();
  },true);
  // The current console creates the export link dynamically, so capture its button too.
  document.getElementById('export').addEventListener('click',event=>{event.preventDefault();event.stopImmediatePropagation();downloadRecording();},true);
});
function downloadRecording(){D.download(JSON.stringify({schema:'microduck-console-recording/v1',demo:true,frames:recordings.length?recordings:[snapshot()]}),'microduck-demo-recording.json');}
window.__microduckDemo={snapshot,config,catalog};
})();
