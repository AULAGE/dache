"""
任务 11 研究原型的单元测试（标准库 unittest）。

  cd raspberrypi/grasp && python3 -m unittest research.test_research
  python3 selftest.py research        # 原有自测 + 这些测试
"""
import io
import os
import random
import sys
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_GRASP = os.path.dirname(_HERE)
if _GRASP not in sys.path:
    sys.path.insert(0, _GRASP)

from research import sim_world as sw  # noqa: E402
from research import association as A  # noqa: E402
from research import outcome_calib as oc  # noqa: E402
from research.hysteresis_id import HysteresisBank  # noqa: E402
import sim as _sim  # noqa: E402


class TestSimWorld(unittest.TestCase):
    def test_play_matches_sim_servo(self):
        """辨识器里的快速迟滞模型和 sim.SimArm._servo_base 完全一致"""
        r = random.Random(1)
        for _ in range(20):
            b, d = r.uniform(0, 4), r.uniform(0, 2.5)
            fake = SimpleNamespace(cfg=SimpleNamespace(SIM_SERVO_DEADBAND_US=d, SIM_SERVO_BACKLASH_US=b),
                                   base_motor=0.0, base_out=0.0)
            m = o = 0.0
            c = 0.0
            for _ in range(50):
                c += r.choice((-1, 1)) * r.uniform(0, 5)
                ref = _sim.SimArm._servo_base(fake, c)
                m, o = sw.play(m, o, c, b, d)
                self.assertAlmostEqual(ref, o, places=9)

    def test_reproducible(self):
        def run(seed):
            w = sw.World(sw.Car(seed), seed)
            w.preload()
            w.new_episode(episode_seed=5)
            w.command({"x": [3.0], "y": [0.5]})
            _, win = w.window(5)
            return [(d.kind, round(d.x_deg, 6), round(d.y, 6)) for f in win for d in f.dets]
        self.assertEqual(run(3), run(3))
        self.assertNotEqual(run(3), run(4))

    def test_lost_motion(self):
        """换向的一小步会被回差吃掉：目标几乎不动；爪子脚永远不动"""
        car = sw.Car(0)
        car.b["x"], car.delta["x"] = 3.0, 0.5
        w = sw.World(car, 0)
        w.preload()
        w.new_episode(n_dist=0, self_active=True, target_view=(0.0, 0.0, 180.0))
        w.command({"x": [5.0]})
        x1 = w.view(w.objs[0], w.t)[0]
        w.command({"x": [3.0]})               # 反向 2° < 回差 3° → 输出不动
        x2 = w.view(w.objs[0], w.t)[0]
        self.assertLess(abs(x2 - x1), 0.6)     # 只剩落点随机误差
        self.assertEqual(w.view(w.objs[1], w.t)[:2], (car.self_view[0], car.self_view[1]))


class TestHysteresis(unittest.TestCase):
    def _synthetic(self, b, d, J, n, seed=0, noise=0.05):
        r = random.Random(seed)
        bank = HysteresisBank.for_axis("x", 25, 17)
        m, o = 0.0, -b / 2
        c = 0.0
        for _ in range(n):
            c += r.choice((-1, 1)) * r.uniform(0.3, 6)
            o0 = o
            m, o = sw.play(m, o, c, b, d)
            bank.apply([c], J * (o - o0) + r.gauss(0, noise))
        return bank

    def test_identify(self):
        bank = self._synthetic(2.0, 1.0, -0.6, 60)
        e = bank.estimate()
        self.assertAlmostEqual(e["b"][0], 2.0, delta=0.3)
        self.assertAlmostEqual(e["delta"][0], 1.0, delta=0.4)
        self.assertAlmostEqual(e["J"][0], -0.6, delta=0.05)

    def test_inverse_compensation(self):
        """真值假设下的逆迟滞：各种情况（同向、换向、小于死区的一小步）一次走到位"""
        b, d, J = 2.5, 1.2, -0.7
        for want in (5.0, -5.0, 0.5, -0.5, 0.3, 12.0):
            for side in (-1, 1):
                m, o = 0.0, side * b / 2
                bank = HysteresisBank.oracle(b, d, J, m, o, 0.3)
                cmds = bank.plan(want)
                o0 = o
                for c in cmds:
                    m, o = sw.play(m, o, c, b, d)
                self.assertAlmostEqual(J * (o - o0), want, delta=0.05, msg=f"want={want} side={side} {cmds}")


class TestAssociation(unittest.TestCase):
    def test_classify_self_and_target(self):
        """升降 / 底座一动：跟着动的是目标，不动的是自身"""
        bank_x, bank_y = HysteresisBank.for_axis("x"), HysteresisBank.for_axis("y")
        tm = A.TrackManager()
        tgt = SimpleNamespace(x=5.0, y=10.0, d=180.0)
        slf = SimpleNamespace(x=-24.0, y=3.0, d=95.0)

        def obs(o):
            return A.WinObs([sw.Det(0, o.x, o.y, o.d, 0, "?")] * 3)
        tm.update([obs(tgt), obs(slf)], None)
        c_y = 0.0
        for k in range(4):
            c_y += 1.0 if k % 2 == 0 else -1.0
            pred = {"x": [(1.0, 0.0, 0.0)], "y": bank_y.predictive([c_y], 40)}
            bank_y.apply([c_y])
            tgt.y += 6.0 * (1.0 if k % 2 == 0 else -1.0) * (1 if k == 0 else 0.9)
            tm.update([obs(tgt), obs(slf)], pred)
        t_tgt = min(tm.tracks, key=lambda t: abs(t.x - 5.0))
        t_slf = min(tm.tracks, key=lambda t: abs(t.x + 24.0))
        self.assertGreater(t_tgt.p["T"], 0.9)
        self.assertGreater(t_slf.p["S"], 0.9)
        self.assertIsNotNone(bank_x)

    def test_lost_motion_is_not_evidence(self):
        """命令被回差吃掉时（预测位移≈0），目标没动不应被当成"自身"的证据"""
        pred = {"x": [(1.0, 0.0, 0.0)], "y": [(1.0, 0.0, 0.0)]}
        dz = {"x": 0.05, "y": -0.3}
        t = A.Track(A.WinObs([sw.Det(0, 0, 0, 180, 0, "?")] * 2))
        A.TrackManager._classify(t, dz, pred)
        self.assertAlmostEqual(t.p["T"] / t.p["S"], A.PRIOR["T"] / A.PRIOR["S"], delta=0.3)

    def test_ghost_rule_like_task(self):
        g = A.GhostRule()
        self.assertFalse(g.update([("y", 10.0, 0.5, 6.0)]))
        self.assertTrue(g.update([("y", 10.0, 0.5, 6.0)]))
        g = A.GhostRule()
        g.update([("y", 10.0, 0.5, 6.0)])
        self.assertFalse(g.update([("y", 10.0, 8.0, 6.0)]))     # 有响应：清零
        self.assertFalse(g.update([("y", 10.0, 0.5, 6.0)]))

    def test_episodes_run(self):
        car = sw.Car(2)
        for method in ("ghost", "p_k230", "p_raw"):
            w = sw.World(car, 2)
            w.preload()
            cs = A.CarState()
            res = [A.run_episode(w, method, cs, episode_seed=i) for i in range(6)]
            for r in res:
                self.assertIn(r["result"], ("commit", "abort", "lost", "timeout"))

    def test_raw_switches_away_from_claw(self):
        """爪子脚一直可见、K230 先锁了爪子：多框概率模型应该换到真目标并完成"""
        ok = 0
        for i in range(8):
            car = sw.Car(10 + i)
            w = sw.World(car, 10 + i)
            w.preload()
            r = A.run_episode(w, "p_raw", A.CarState(), n_dist=0, self_active=True, episode_seed=i)
            ok += r["result"] == "commit" and r["kind"] == "target"
        self.assertGreaterEqual(ok, 7)


class TestOutcome(unittest.TestCase):
    def test_p_success(self):
        self.assertAlmostEqual(oc.p_success(0, 0, 8, 3), oc.phi(8 / 3))
        self.assertAlmostEqual(oc.p_success(2, 5, 8, 3), oc.p_success(8, 5, 8, 3))

    def test_converges(self):
        errs = []
        for s in range(6):
            rng = random.Random(s)
            r_true = rng.uniform(-6, 6)
            res = oc.run_calibration(r_true, "ig", 40, rng, checkpoints=(40,))
            errs.append(res["err"][40])
        self.assertLess(sum(errs) / len(errs), 1.5)

    def test_exploit_learns_nothing_when_always_success(self):
        """纯利用：一直瞄准 0 且一直成功，只知道 |r| 不大，完全不知道偏哪边（这就是要主动探测的原因）"""
        post = oc.ResidualPosterior()
        for _ in range(20):
            post.update(0.0, True)
        self.assertLess(abs(post.mean()), 0.2)
        p_right = sum(w for w, r in zip(post.w, oc.GRID) if r > 0.5)
        p_left = sum(w for w, r in zip(post.w, oc.GRID) if r < -0.5)
        self.assertAlmostEqual(p_right, p_left, delta=0.02)
        self.assertGreater(p_right, 0.25)


class TestRunExperiments(unittest.TestCase):
    def test_quick(self):
        from research import run_experiments as R
        with redirect_stdout(io.StringIO()):
            out = R.main(["--seed", "0", "--quick"])
        txt = "\n".join(out)
        for h in ("## 1 ", "## 2 ", "## 3 ", "## 4 ", "仿真"):
            self.assertIn(h, txt)


if __name__ == "__main__":
    unittest.main()
