"""Drives chat_node + behavior_ctrl through the Phase 2 scenarios."""
import sys, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ugv_interface.msg import Say, Transcript
from ugv_interface.srv import Record
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import os, wave

class Checker(Node):
    def __init__(self):
        super().__init__('e2e_checker')
        self.says, self.twists = [], []
        self.x = 0.0; self.vx = 0.0; self.lock = threading.Lock()
        self.create_subscription(Say, '/voice/say', lambda m: self.says.append((time.monotonic(), m)), 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tr_pub = self.create_publisher(Transcript, '/voice/transcript', 10)
        self.create_timer(0.05, self.tick)
        self._last = time.monotonic()
        self.records = []
        self.create_service(Record, 'voice/record', self.record_cb)
        # Fake Nav2: accepts goals, sits on them until cancelled.
        self.nav_goals, self.nav_cancels = [], []
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.nav_server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                       execute_callback=self.nav_execute,
                                       callback_group=ReentrantCallbackGroup(),
                                       cancel_callback=lambda gh: (self.nav_cancels.append(time.monotonic()), CancelResponse.ACCEPT)[1])
    def nav_execute(self, gh):
        self.nav_goals.append(time.monotonic())
        while not gh.is_cancel_requested and time.monotonic() - self.nav_goals[-1] < 20:
            time.sleep(0.05)
        if gh.is_cancel_requested:
            gh.canceled()
        else:
            gh.succeed()
        return NavigateToPose.Result()
    def record_cb(self, req, resp):
        # Stand-in for the ear: write a short silent wav where asked.
        self.records.append((req.duration_s, req.path))
        os.makedirs(os.path.dirname(req.path), exist_ok=True)
        with wave.open(req.path, 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
            w.writeframes(b'\x00\x00' * int(16000 * req.duration_s))
        resp.success = True; resp.path = req.path; resp.message = 'ok'
        return resp
    def cmd_cb(self, m):
        self.twists.append((time.monotonic(), m)); self.vx = m.linear.x
    def tick(self):
        now = time.monotonic(); dt = now - self._last; self._last = now
        self.x += self.vx * dt
        o = Odometry(); o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'; o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x = self.x; o.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(o)
        p = PoseStamped(); p.header.frame_id = 'map'; p.pose.orientation.w = 1.0
        self.pose_pub.publish(p)
    def say(self, text, stop=False):
        m = Transcript(); m.text = text; m.stop_word = stop; self.tr_pub.publish(m)

class Hit:
    def __init__(self, ts, m): self.ts, self.m = ts, m
    def __getattr__(self, k): return getattr(self.m, k)

results = []
def check(name, cond): results.append((name, cond)); print(('PASS ' if cond else 'FAIL ') + name, flush=True)

def wait_say(n, pred, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        for ts, m in n.says[:]:
            if ts > t0 - 0.05 and pred(m): return Hit(ts, m)
        time.sleep(0.05)
    return None

def wait_cmd(n, pred, timeout=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        for ts, m in n.twists[:]:
            if ts > t0 and pred(m): return ts
        time.sleep(0.02)
    return None

def main():
    rclpy.init(); n = Checker()
    ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    time.sleep(3.0)   # let behavior_ctrl/chat_node discover each other

    # 1. plain conversation
    n.say('hello robot')
    m = wait_say(n, lambda m: 'Hello there' in m.text)
    check('conversation streamed to /voice/say', m is not None and m.key == 'chat_reply')
    time.sleep(1.0)

    # 2. motion tool: model speaks, tool dispatched, wheels move after delay, stop word halts
    t_send = time.monotonic(); n.say('go forward a little')
    m = wait_say(n, lambda m: m.text.startswith('Rolling forward'))
    check('model ack spoken before motion', m is not None)
    first_cmd = wait_cmd(n, lambda t: t.linear.x > 0.0)
    check('cmd_vel forward after speak-then-act', first_cmd is not None)
    if first_cmd:
        check('cmd_vel came after ack (speak-then-act delay honoured)', m is not None and first_cmd >= m.ts + 0.4)
        check('narration of tool result spoken', any(ts > t_send and 'done' in m.text.lower() for ts, m in n.says) or wait_say(n, lambda m: 'done' in m.text.lower(), 4.0) is not None)
        n.say('stop', stop=True); t_stop = time.monotonic()
        t_zero = wait_cmd(n, lambda t: t.linear.x == 0.0 and t.angular.z == 0.0, 2.0)
        check('stop word -> zero twist within 300 ms', t_zero is not None and t_zero - t_stop < 0.3)
        s = wait_say(n, lambda m: m.key == 'estop', 2.0)
        check('"Stopping." spoken at safety priority', s is not None and s.priority == Say.PRIORITY_SAFETY)
        time.sleep(0.5)
        check('no further forward motion after stop', wait_cmd(n, lambda t: t.linear.x > 0.0, 1.5) is None)
    time.sleep(1.0)

    # 3. clamp: distance 999 -> too_far template, 1.0 m motion
    n.say('call move with distance 999')
    m = wait_say(n, lambda m: m.key == 'too_far')
    check('999 m clamped: too_far refusal spoken', m is not None and '1 meters' in m.text)
    check('clamped motion still dispatched (drives)', wait_cmd(n, lambda t: t.linear.x > 0.0) is not None)
    n.say('stop', stop=True); time.sleep(1.0)

    # 4. unknown tool
    n.twists.clear(); n.say('please fly')
    m = wait_say(n, lambda m: m.key == 'unknown')
    check('unknown tool refused with spoken refusal', m is not None)
    check('unknown tool moved nothing', wait_cmd(n, lambda t: t.linear.x != 0 or t.angular.z != 0, 2.0) is None)

    # 5. compound: two calls in one message -> one honoured
    n.twists.clear(); t_c = time.monotonic(); n.say('go forward twice')
    check('compound: second call refused one_at_a_time', wait_say(n, lambda m: m.key == 'one_at_a_time') is not None)
    check('compound: first call dispatched', wait_cmd(n, lambda t: t.linear.x > 0.0) is not None or any(ts > t_c and m.linear.x > 0 for ts, m in n.twists))
    time.sleep(3.5)   # 0.3 m drive finishes
    check('compound: second call (spin) NOT dispatched', not any(ts > t_c and m.angular.z != 0 for ts, m in n.twists))
    n.say('stop', stop=True); time.sleep(1.0)

    # 6. battery tool + narration
    n.say('how is your battery')
    check('battery tool -> model narrates', wait_say(n, lambda m: 'done' in m.text.lower(), 5.0) is not None)

    # 7. go_to_point needs confirmation; 'no' cancels
    n.say('go to point a')
    check('go_to_point asks for confirmation', wait_say(n, lambda m: m.key == 'confirm_go_to_point') is not None)
    n.say('no')
    check('deny cancels navigation', wait_say(n, lambda m: m.key == 'nav_cancelled') is not None)

    # 7a. Nav2 cancellation through the estop path (Phase 4 check)
    n.say('save point a')
    check('nav: save_point acked', wait_say(n, lambda m: m.key == 'ack_save_point') is not None)
    time.sleep(1.0)
    n.say('go to point a')
    check('nav: confirmation asked', wait_say(n, lambda m: m.key == 'confirm_go_to_point') is not None)
    n.say('yes')
    check('nav: confirmed', wait_say(n, lambda m: m.key == 'nav_confirmed') is not None)
    t0 = time.monotonic()
    while not n.nav_goals and time.monotonic() - t0 < 8: time.sleep(0.05)
    check('nav: NavigateToPose goal reached the (fake) Nav2 server', bool(n.nav_goals))
    time.sleep(0.5)
    n.say('stop', stop=True); t_stop = time.monotonic()
    t0 = time.monotonic()
    while not n.nav_cancels and time.monotonic() - t0 < 3: time.sleep(0.02)
    check('nav: stop word cancels the Nav2 goal (<1 s)', n.nav_cancels and n.nav_cancels[-1] - t_stop < 1.0)
    time.sleep(1.0)

    # 7b. record_replay: countdown, record service, wav playbacks with speeds
    n.records.clear(); t_rec = time.monotonic(); n.say('record me and play it back like a chipmunk')
    check('record: countdown spoken', wait_say(n, lambda m: m.key == 'record_countdown') is not None)
    t0 = time.monotonic()
    while not n.records and time.monotonic() - t0 < 12: time.sleep(0.1)
    check('record: ear service called with clamped duration', n.records and n.records[0][0] == 3.0)
    wavs = []
    t0 = time.monotonic()
    while len(wavs) < 2 and time.monotonic() - t0 < 8:
        wavs = [m for ts, m in n.says if m.key == 'record_playback_wav']; time.sleep(0.1)
    check('record: two wav playbacks queued', len(wavs) == 2)
    check('record: speeds 1.0 then chipmunk 1.3', [round(m.speed_factor, 2) for m in wavs] == [1.0, 1.3])
    check('record: delete_after only on the last playback', [m.delete_after for m in wavs] == [False, True])
    check('record: wav path exists until the mouth deletes it', wavs and os.path.exists(wavs[0].wav_path))
    check('record: model narrates afterwards', any(ts > t_rec and 'done' in m.text.lower() for ts, m in n.says) or wait_say(n, lambda m: 'done' in m.text.lower(), 5.0) is not None)
    for m in wavs:
        try: os.unlink(m.wav_path)   # the mouth is not running in this harness
        except OSError: pass

    # 7c. record_replay refused while moving
    n.records.clear(); n.say('go forward a little')
    check('pre: driving', wait_cmd(n, lambda t: t.linear.x > 0.0) is not None)
    n.say('record me')
    check('record while moving -> refused', wait_say(n, lambda m: m.key == 'record_refused_moving') is not None)
    time.sleep(1.0)
    check('record while moving -> service never called', not n.records)
    n.say('stop', stop=True); time.sleep(1.5)

    # 8. offline fallback: kill the fake daemon (parent shell does it on this marker)
    print('KILL_OLLAMA', flush=True); time.sleep(2.0)
    n.twists.clear(); n.say('go forward')
    check('offline: chat_offline template spoken', wait_say(n, lambda m: m.key == 'chat_offline', 8.0) is not None)
    check('offline: RuleBrain ack spoken', wait_say(n, lambda m: m.key == 'ack_move_forward', 4.0) is not None)
    check('offline: RuleBrain motion dispatched', wait_cmd(n, lambda t: t.linear.x > 0.0) is not None)
    n.say('stop', stop=True)
    check('offline: stop still halts', wait_cmd(n, lambda t: t.linear.x == 0.0, 2.0) is not None)

    fails = [r for r in results if not r[1]]
    print('\n%d/%d checks passed' % (len(results) - len(fails), len(results)), flush=True)
    rclpy.shutdown(); sys.exit(1 if fails else 0)

main()
