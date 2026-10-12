"""Calibration installation and runtime consistency, without a physical robot."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import tomllib
import unittest
from unittest.mock import patch
import math
import runtime
from configure import dumps
from service_control import validate,ControlError
from service_data import adapt_service

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('restore100',ROOT/'rebuild-config.py')
restore=importlib.util.module_from_spec(spec);spec.loader.exec_module(restore)

class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.base=Path(self.temp.name)
        self.path=self.base/'hd1910-calibration.toml'
        self.seed=tomllib.loads((ROOT/'hd1910-calibration.robot.toml').read_text())
        for name in ('hd1910-calibration.template.toml','hd1910-calibration.robot.toml'):
            (self.base/name).write_bytes((ROOT/name).read_bytes())
    def tearDown(self):self.temp.cleanup()
    def write(self,data):self.path.write_text(dumps(data))
    def test_snapshot_is_exact_current_walk_model_robot_record_not_old_training_zero(self):
        contract=json.loads((ROOT.parent/'policies/walk/deployment-contract.json').read_text())
        self.assertEqual(self.seed,contract['hardware_calibration']['data'])
        self.assertEqual(self.seed['joints'][8]['zero_raw'],1599)
        self.assertEqual(self.seed['joints'][6]['zero_raw'],1742)
    def test_empty_install_and_confirmed_placeholder_imu_restore_known_assembled_values(self):
        for present in (False,True):
            if present:
                template=tomllib.loads((ROOT/'hd1910-calibration.template.toml').read_text())
                template['imu_mount_verified']=True;self.write(template)
            result=restore.prepare_calibration(self.path,self.base)
            self.assertEqual(result,self.seed)
    def test_local_calibration_and_later_mouth_or_imu_edits_survive_reinstall(self):
        data=copy.deepcopy(self.seed);data['joints'][8]['zero_raw']=1620
        data['joints'][9]['max_rad']=math.radians(12)
        data['imu_mount_quat']=[1.,0.,0.,0.];data['motion_enabled']=True
        self.write(data);self.assertEqual(restore.prepare_calibration(self.path,self.base),data)
    def test_partial_local_and_saved_rows_win_before_known_snapshot(self):
        data=copy.deepcopy(self.seed);data['joints'][8]['zero_raw']=1650
        data['joints'][0]['calibrated']=False
        saved=copy.deepcopy(self.seed);saved['joints'][0]['zero_raw']=2050
        restore.saved_path(self.path).write_text(dumps(saved));self.write(data)
        result=restore.prepare_calibration(self.path,self.base)
        self.assertEqual(result['joints'][8]['zero_raw'],1650)
        self.assertEqual(result['joints'][0]['zero_raw'],2050)
    def test_ordinary_recovery_never_reapplies_seed_after_edit(self):
        data=copy.deepcopy(self.seed);data['joints'][8]['zero_raw']=1640
        data['imu_mount_quat']=[1.,0.,0.,0.];self.write(data)
        self.assertEqual(restore.source_data(self.path,'calibration',self.base),data)
    def test_install_preserves_local_unconfirmed_imu_without_upgrade_gate(self):
        data=copy.deepcopy(self.seed);data['imu_mount_verified']=False;data['units_verified']=False
        self.write(data)
        result=restore.prepare_calibration(self.path,self.base)
        self.assertFalse(result['units_verified'])
        self.assertEqual(result['joints'],data['joints'])

    def test_runtime_checks_real_file_path_zero_limits_and_imu(self):
        self.write(self.seed)
        report={'calibration_path':str(self.path),'calibration':self.seed}
        cfg={'bus':{'feetech_calibration':str(self.path)}}
        with patch.object(runtime,'CONFIG',self.base/'robotd.toml'),patch.object(runtime,'config',return_value=cfg),patch.object(runtime,'rpc',return_value=report):
            self.assertTrue(runtime.verify_calibration()['calibration_verified'])
            for mutation in ('path','zero','imu'):
                bad=copy.deepcopy(report)
                if mutation=='path':bad['calibration_path']='/wrong.toml'
                elif mutation=='zero':bad['calibration']['joints'][8]['zero_raw']=568
                else:bad['calibration']['imu_mount_quat']=[1,0,0,0]
                with patch.object(runtime,'rpc',return_value=bad),self.assertRaises(ValueError):runtime.verify_calibration()
    def checker(self):
        spec=importlib.util.spec_from_file_location('calibration_sources',ROOT/'calibration-config.py')
        helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
        return helper

    def test_a_module_source_needs_its_own_verified_mount(self):
        helper=self.checker()
        helper.validate(self.seed)
        with self.assertRaisesRegex(ValueError,'bno08x IMU 安装方向尚未确认'):helper.validate(self.seed,'bno08x')
        data=copy.deepcopy(self.seed);data['imu_mount_verified']=False
        data['imu_mounts']={'bno08x':{'quat':[.5,.5,.5,.5],'verified':True}}
        helper.validate(data,'bno08x')
        with self.assertRaisesRegex(ValueError,'IMU 安装方向尚未确认'):helper.validate(data)
        data['imu_mounts']['lsm6dsv16x']={'quat':[2.,0.,0.,0.],'verified':False}
        with self.assertRaisesRegex(ValueError,'lsm6dsv16x IMU 安装四元数未归一化'):helper.validate(data,'bno08x')
        data['imu_mounts']={'bmi088':{'quat':[1.,0.,0.,0.]}}
        with self.assertRaisesRegex(ValueError,'imu_mounts'):helper.validate(data,'bno08x')

    def test_the_check_reads_the_source_robotd_is_configured_with(self):
        import subprocess,sys
        data=copy.deepcopy(self.seed)
        self.write(data)
        (self.base/'robotd.toml').write_text('[trunk_imu]\nsource = "lsm6dsv16x"\ni2c_address = 0x6A\n')
        check=lambda:subprocess.run([sys.executable,str(ROOT/'calibration-config.py'),'check',str(self.path)],capture_output=True,text=True)
        result=check()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('lsm6dsv16x',result.stderr)
        data['imu_mounts']={'lsm6dsv16x':{'quat':[1.,0.,0.,0.],'verified':True}};self.write(data)
        self.assertEqual(check().returncode,0)

    def test_install_keeps_module_mounts_and_never_seeds_them(self):
        data=copy.deepcopy(self.seed);data['imu_mounts']={'bno08x':{'quat':[.5,.5,.5,.5],'verified':True}}
        self.write(data)
        self.assertEqual(restore.prepare_calibration(self.path,self.base)['imu_mounts'],data['imu_mounts'])
        restore.saved_path(self.path).write_text(dumps(data))
        template=tomllib.loads((ROOT/'hd1910-calibration.template.toml').read_text());self.write(template)
        result=restore.prepare_calibration(self.path,self.base)
        self.assertEqual(result['imu_mounts'],data['imu_mounts'],'restored from the saved copy')
        restore.saved_path(self.path).unlink();self.write(template)
        self.assertNotIn('imu_mounts',restore.prepare_calibration(self.path,self.base))

    def test_a_module_robot_never_gets_a_bus_mount_it_did_not_verify(self):
        # The snapshot's mount is a bus board's. Backfilling it as verified on a module robot
        # would let motion run on a never-checked board the day the source goes back to bus.
        data=copy.deepcopy(self.seed);data['imu_mount_verified']=False;data['imu_mount_quat']=[1.,0.,0.,0.]
        data['imu_mounts']={'bno08x':{'quat':[.5,.5,.5,.5],'verified':True}};data['motion_enabled']=True
        self.write(data)
        result=restore.prepare_calibration(self.path,self.base,'bno08x')
        self.assertFalse(result['imu_mount_verified'])
        self.assertEqual(result['imu_mount_quat'],[1.,0.,0.,0.])
        self.assertTrue(result['motion_enabled'],'the wired mount is verified; nothing to stop')
        data['imu_mounts']['bno08x']['verified']=False;self.write(data)
        self.assertFalse(restore.prepare_calibration(self.path,self.base,'bno08x')['motion_enabled'])
        # The same file on a bus robot is still restored from the snapshot, as before.
        self.assertTrue(restore.prepare_calibration(self.path,self.base)['imu_mount_verified'])

    def test_prepare_reads_the_source_from_the_staged_params(self):
        profile=tomllib.loads((ROOT/'robotd-profile.toml').read_text())
        profile['trunk_imu']={'source':'lsm6dsv16x','i2c_address':0x6A}
        params=self.base/'robotd.toml';params.write_text(dumps(profile))
        data=copy.deepcopy(self.seed);data['imu_mount_verified']=False
        data['imu_mounts']={'lsm6dsv16x':{'quat':[1.,0.,0.,0.],'verified':True}};self.write(data)
        staged=self.base/'staged';staged.mkdir()
        restore.prepare(staged,params,self.path,self.base)
        result=tomllib.loads((staged/'hd1910-calibration.toml').read_text())
        self.assertFalse(result['imu_mount_verified'])
        self.assertTrue(result['imu_mounts']['lsm6dsv16x']['verified'])

    def test_runtime_checks_module_mounts_against_the_saved_file(self):
        data=copy.deepcopy(self.seed);data['imu_mounts']={'bno08x':{'quat':[.5,.5,.5,.5],'verified':True}}
        self.write(data)
        report={'calibration_path':str(self.path),'calibration':copy.deepcopy(data)}
        cfg={'bus':{'feetech_calibration':str(self.path)}}
        with patch.object(runtime,'CONFIG',self.base/'robotd.toml'),patch.object(runtime,'config',return_value=cfg),patch.object(runtime,'rpc',return_value=report):
            self.assertTrue(runtime.verify_calibration()['calibration_verified'])
            for mutation in ('missing','unverified','quat'):
                bad=copy.deepcopy(report)
                if mutation=='missing':del bad['calibration']['imu_mounts']
                elif mutation=='unverified':bad['calibration']['imu_mounts']['bno08x']['verified']=False
                else:bad['calibration']['imu_mounts']['bno08x']['quat']=[1,0,0,0]
                with patch.object(runtime,'rpc',return_value=bad),self.assertRaisesRegex(ValueError,'模块 IMU'):runtime.verify_calibration()

    def test_saved_count_does_not_require_a_live_raw_sample(self):
        result={'channels':{'bus':{'status':'live','age_ms':0},'state':{'status':'live'}},'servos':[{'id':r['id'],'name':r['name'],'index':i} for i,r in enumerate(self.seed['joints'])],'imu':{}}
        adapt_service(result,{'mode':'commissioning','build':runtime.BUILD,'phase':'ready'}, {'calibration':self.seed},0)
        self.assertEqual(result['saved_calibrated_count'],15)
        self.assertEqual(result['calibrated_count'],0)
        self.assertFalse(result['pose_calibrated'])

if __name__=='__main__':unittest.main()
