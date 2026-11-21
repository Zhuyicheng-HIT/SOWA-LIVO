#!/usr/bin/env python3
"""Persistent Gazebo service-only transport process: no sensor callback GIL contention."""
import json,sys
from google.protobuf import text_format
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
node=Node()
for line in sys.stdin:
 try:
  command=json.loads(line);typ={'gz.msgs.Pose':Pose,'gz.msgs.WorldControl':WorldControl}[command['type']]
  req=typ();text_format.Parse(command['text'],req)
  ok,reply=node.request(command['service'],req,typ,Boolean,10000)
  print(json.dumps({'ok':bool(ok and reply.data),'transport':bool(ok),'reply':str(reply)}),flush=True)
 except Exception as exc:print(json.dumps({'ok':False,'error':repr(exc)}),flush=True)
