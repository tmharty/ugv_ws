"""Fake Ollama /api/chat for the chat_node end-to-end check. Streams
newline-delimited JSON exactly like the daemon, including tool_calls."""
import json, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

def chunk(content='', tool_calls=None, done=False):
    m = {'role': 'assistant', 'content': content}
    if tool_calls: m['tool_calls'] = tool_calls
    return json.dumps({'model': 'fake', 'message': m, 'done': done}) + '\n'

def call(name, **args):
    return {'function': {'name': name, 'arguments': args}}

def script(req):
    msgs = req['messages']; tools = req.get('tools'); last = msgs[-1]
    if last['role'] == 'tool':
        return [chunk('Okay, '), chunk('done.')]
    t = last['content'].lower()
    if not tools:
        return [chunk('No tools offered.')]
    if '999' in t:
        return [chunk(tool_calls=[call('move', direction='forward', distance_m=999)])]
    if 'fly' in t:
        return [chunk('Taking off! ', tool_calls=[call('fly', altitude=100)])]
    if 'twice' in t:   # compound: two calls in one message
        return [chunk(tool_calls=[call('move', direction='forward', distance_m=0.3),
                                  call('spin_around')])]
    if 'forward' in t:
        return [chunk('Rolling '), chunk('forward. '),
                chunk(tool_calls=[call('move', direction='forward', distance_m=0.4)])]
    if 'record' in t:
        return [chunk('Say something! ', tool_calls=[call('record_replay', duration_s=3, speeds=[1.0, 'chipmunk'])])]
    if 'battery' in t:
        return [chunk(tool_calls=[call('battery_status')])]
    if 'point' in t:
        return [chunk(tool_calls=[call('go_to_point', point='A')])]
    return [chunk('Hello there, '), chunk('I am a robot.')]

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        print('REQ tools=%s last=%s' % (bool(body.get('tools')),
              body['messages'][-1]), flush=True)
        self.send_response(200); self.send_header('Content-Type', 'application/x-ndjson')
        self.end_headers()
        for c in script(body): self.wfile.write(c.encode()); self.wfile.flush()
        self.wfile.write(chunk(done=True).encode())

ThreadingHTTPServer(('127.0.0.1', int(sys.argv[1]) if len(sys.argv) > 1 else 11434), H).serve_forever()
