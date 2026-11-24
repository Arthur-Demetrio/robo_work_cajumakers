"""Independent protobuf decoder verifies TensorBoard scalar wire compatibility."""
import struct,tempfile,unittest
from pathlib import Path
from google.protobuf import descriptor_pb2,descriptor_pool,message_factory
from robo_rl_v2.events import ScalarEvents

class EventSchemaTest(unittest.TestCase):
    def test_scalar_decodes_as_official_event_schema(self):
        file=descriptor_pb2.FileDescriptorProto(name='event_minimal.proto',package='tensorflow',syntax='proto3')
        value=file.message_type.add(name='Value')
        value.field.add(name='tag',number=1,type=9,label=1)
        value.field.add(name='simple_value',number=2,type=2,label=1)
        summary=file.message_type.add(name='Summary')
        summary.field.add(name='value',number=1,type=11,label=3,type_name='.tensorflow.Value')
        event=file.message_type.add(name='Event')
        event.field.add(name='wall_time',number=1,type=1,label=1)
        event.field.add(name='step',number=2,type=3,label=1)
        event.field.add(name='file_version',number=3,type=9,label=1)
        event.field.add(name='summary',number=5,type=11,label=1,type_name='.tensorflow.Summary')
        pool=descriptor_pool.DescriptorPool();pool.Add(file)
        Event=message_factory.MessageFactory(pool).GetPrototype(pool.FindMessageTypeByName('tensorflow.Event'))
        with tempfile.TemporaryDirectory() as tmp:
            writer=ScalarEvents(tmp);writer.scalar('eval/success',.75,25000);writer.close()
            data=next(Path(tmp).iterdir()).read_bytes();offset=0;events=[]
            while offset<len(data):
                n=struct.unpack('<Q',data[offset:offset+8])[0]
                events.append(Event.FromString(data[offset+12:offset+12+n]));offset+=16+n
            self.assertEqual(events[0].file_version,'brain.Event:2')
            self.assertEqual(events[1].step,25000)
            self.assertEqual(events[1].summary.value[0].tag,'eval/success')
            self.assertAlmostEqual(events[1].summary.value[0].simple_value,.75)
