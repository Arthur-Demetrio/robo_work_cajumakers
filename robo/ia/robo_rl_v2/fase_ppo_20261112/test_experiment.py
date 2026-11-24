import unittest,tempfile,copy
from experiment import *
from stable_baselines3.common.distributions import DiagGaussianDistribution
class PilotTests(unittest.TestCase):
 def test_projection_only_angular_every_adam_step(self):
  m=PPO.load(ROOT/'origem.zip',device='cpu');p=m.policy;before=p.log_std.detach().clone();old=copy.deepcopy(p.optimizer.state_dict())
  hook=AngularProjection(p)
  self.assertTrue(torch.equal(p.log_std[:2],before[:2]));self.assertEqual(float(p.log_std[2]),0.)
  for k,v in old['state'].items():
   for name,t in v.items():self.assertTrue(torch.equal(t,p.optimizer.state_dict()['state'][k][name]))
  for _ in range(5):
   p.optimizer.zero_grad();(-p.log_std[2]*100).backward();p.optimizer.step();self.assertLessEqual(float(p.log_std[2]),0.)
  self.assertEqual(hook.steps,5);self.assertGreater(hook.count,1);self.assertNotEqual(hook.state()['adam_angular_m'],0.);hook.handle.remove()
 def test_raw_probabilities_not_clipped_gaussian_density(self):
  torch.manual_seed(8);d=DiagGaussianDistribution(3).proba_distribution(torch.tensor([[1.4,-.3,2.]]).repeat(4000,1),torch.tensor([-.5,-.3,1.]))
  a=d.sample();old=d.log_prob(a);new=d.log_prob(a.clone())
  self.assertTrue(torch.equal(old,new));self.assertTrue(torch.equal(torch.exp(new-old),torch.ones_like(old)))
  self.assertGreater(float((old-d.log_prob(a.clamp(-1,1))).abs().mean()),.1)
 def test_kl_entropy(self):
  torch.manual_seed(5);old=torch.distributions.Normal(torch.zeros(80000,3),torch.ones(80000,3));new=torch.distributions.Normal(torch.ones(80000,3)*.2,torch.ones(80000,3)*1.1)
  a=old.sample();lr=(new.log_prob(a)-old.log_prob(a)).sum(-1)
  self.assertAlmostEqual(float((torch.exp(lr)-1-lr).mean()),float(torch.distributions.kl_divergence(old,new).sum(-1).mean()),delta=.003)
  d=DiagGaussianDistribution(3).proba_distribution(torch.zeros(1,3),torch.zeros(3))
  self.assertAlmostEqual(float(d.entropy()[0]),3*.5*math.log(2*math.pi*math.e),places=5)
 def test_evaluation_rng_and_seed_parity(self):
  m=PPO.load(ROOT/'origem.zip',device='cpu');random.seed(42);np.random.seed(42);torch.manual_seed(42)
  expected=(random.random(),np.random.rand(),torch.rand(1))
  random.seed(42);np.random.seed(42);torch.manual_seed(42);a=evaluate(m,0,[3700000000],True)
  actual=(random.random(),np.random.rand(),torch.rand(1))
  self.assertEqual(actual[:2],expected[:2]);self.assertTrue(torch.equal(actual[2],expected[2]));self.assertEqual(a,evaluate(m,0,[3700000000],True))
 def test_real_rollout_optimizer_restore_update_and_reload(self):
  with tempfile.TemporaryDirectory() as tmp:
   env=DummyVecEnv([StageEnv,StageEnv]);m=PPO.load(ROOT/'origem.zip',env=env,device='cpu');other=PPO.load(ROOT/'origem.zip',device='cpu')
   for k,v in m.policy.state_dict().items():self.assertTrue(torch.equal(v,other.policy.state_dict()[k]))
   for k,v in m.policy.optimizer.state_dict()['state'].items():
    for name,t in v.items():self.assertTrue(torch.equal(t,other.policy.optimizer.state_dict()['state'][k][name]))
   hook=AngularProjection(m.policy);cb=Telemetry(Path(tmp));m.set_logger(configure(tmp,[]));m.set_random_seed(4201)
   m._setup_learn(64,cb,reset_num_timesteps=False);cb.on_training_start({}, {})
   m.rollout_buffer=RolloutBuffer(32,m.observation_space,m.action_space,device='cpu',gamma=m.gamma,gae_lambda=m.gae_lambda,n_envs=2)
   self.assertTrue(m.collect_rollouts(env,cb,m.rollout_buffer,32));self.assertTrue((np.abs(m.rollout_buffer.actions)>1).any())
   for data in m.rollout_buffer.get(64):
    with torch.no_grad():_,logp,_=m.policy.evaluate_actions(data.observations,data.actions)
    self.assertTrue(torch.allclose(logp,data.old_log_prob,atol=1e-5))
   m.train();self.assertEqual(hook.steps,10);self.assertTrue(np.isfinite(m.logger.name_to_value['train/approx_kl']))
   self.assertTrue(all(torch.isfinite(p).all() for p in m.policy.parameters()));self.assertLessEqual(float(m.policy.log_std[2]),0.)
   m.save(Path(tmp)/'checkpoint.zip');loaded=PPO.load(Path(tmp)/'checkpoint.zip',device='cpu');self.assertTrue(torch.equal(m.policy.log_std,loaded.policy.log_std))
   hook.handle.remove();env.close()
 def test_orchestrated_training_checkpoint_and_report_records(self):
  import experiment,shutil
  from unittest.mock import patch
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);shutil.copy2(ROOT/'origem.zip',root/'origem.zip')
   original_evaluate=experiment.evaluate
   def small_evaluate(model,stage,seeds,stochastic=False,full=False):
    return original_evaluate(model,stage,list(seeds)[:1],stochastic,full)
   with patch.object(experiment,'ROOT',root),patch.object(experiment,'evaluate',side_effect=small_evaluate),patch('report.generate'):
    experiment.train('variancia_angular',4201,budget=64)
   run=root/'runs/variancia_angular_s4201';status=json.loads((run/'status.json').read_text())
   self.assertTrue(status['completed']);self.assertEqual(status['new_steps'],64)
   rec=json.loads((run/'evaluations/000064.json').read_text())
   self.assertIn('train/approx_kl',rec['optimizer']);self.assertIn('audit/grad_norm_actor',rec['optimizer'])
   self.assertEqual(rec['projection']['optimizer_steps'],10);self.assertIn('unseen',rec['metrics'])
 def test_exact_budget(self):
  done=0;boundaries=[]
  while done<250000:
   end=min((done//25000+1)*25000,250000);n=min(256,(end-done)//2);self.assertGreater(n,0);done+=n*2
   if done==end:boundaries.append(done)
  self.assertEqual(boundaries,list(range(25000,250001,25000)))
if __name__=='__main__':unittest.main()
