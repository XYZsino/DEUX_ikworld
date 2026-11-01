import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest
from scan_bbox_gate import matching_boxes


def message(name='bread_bag', xy=(0,0,60,50), stamp=10., request='scan'):
    return NS(header=NS(stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
              request_id=request,boxes=[NS(class_name=name,xyxy=xy,track_id='target')])


class BboxGateTests(unittest.TestCase):
    def match(self,msg,arrival=10.,now=10.1):
        return matching_boxes(msg,'scan',arrival,now,.3,3000,300)

    def test_either_class_at_threshold(self):
        self.assertTrue(self.match(message()))
        self.assertTrue(self.match(message('barcode',(0,0,20,15))))
        self.assertTrue(matching_boxes(message(xy=(0,0,25,40)),'scan',10.,10.1,.3,1000,300))
        self.assertFalse(matching_boxes(message(xy=(0,0,24,40)),'scan',10.,10.1,.3,1000,300))

    def test_small_other_invalid_boxes_rejected(self):
        for name,xy in [('bread_bag',(0,0,59,50)),('barcode',(0,0,20,14)),
                        ('person',(0,0,200,200)),('bread_bag',(60,0,0,50)),
                        ('bread_bag',(0,0,float('nan'),50))]:
            self.assertFalse(self.match(message(name,xy)))

    def test_old_wrong_request_stale_and_future_rejected(self):
        self.assertFalse(self.match(message(stamp=9.99)))
        self.assertFalse(self.match(message(request='previous')))
        self.assertFalse(self.match(message(),now=10.5))
        self.assertFalse(self.match(message(stamp=10.2),now=10.1))

    def test_async_timeout_and_success_clear_context(self):
        source=ast.parse(Path(__file__).with_name('lookup_bt_node.py').read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='LookupTaskNode')
        method=next(n for n in cls.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='scan_bbox_gate')
        class Missing(ValueError):pass
        async def pause(node,delay):
            if node.accept:node.scan_bbox_evidence['matches']=[{'area_px2':3000}]
            await asyncio.sleep(.001)
        scope=dict(time=time,json=__import__('json'),pause=pause,ScanObjectMissing=Missing)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'gate','exec'),scope)
        for accept in (False,True):
            node=NS(accept=accept,set_state=lambda s:None,now=lambda:10.,
                    perception_request=NS(request_id='scan'),guard=lambda *a:None,
                    get_parameter=lambda p:NS(value=.01 if p=='scan_bbox_wait_s' else 300.),
                    get_logger=lambda:NS(info=lambda s:None))
            if accept:asyncio.run(scope['scan_bbox_gate'](node,None,None))
            else:
                with self.assertRaises(Missing):asyncio.run(scope['scan_bbox_gate'](node,None,None))
            self.assertEqual(node.scan_bbox_evidence['passed'],accept)
            self.assertIsNone(node.scan_bbox_context)

    def test_missing_returns_ready_then_waits_next_cycle_without_placement_recovery(self):
        source=ast.parse(Path(__file__).with_name('lookup_bt_node.py').read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='LookupTaskNode')
        method=next(n for n in cls.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='execute')
        class Missing(ValueError):pass
        calls=[]
        node=NS(repeat_enabled=True,estop=False,recover_before_next=False,config_version=1,
            prepare_client='prepare',get_parameter=lambda p:NS(value=0),
            publish_repeat_status=lambda:None,publish_status=lambda:None,
            set_state=lambda s:calls.append(s))
        async def cycle(goal):
            calls.append('cycle')
            if calls.count('cycle')==1:raise Missing('no bbox')
            node.repeat_enabled=False
            return NS(success=True)
        async def hold(value):calls.append(('hold',value))
        async def child(*args):calls.append('ready')
        node.execute_cycle=cycle;node.hold_perception=hold;node.child_call=child
        scope=dict(uuid4=__import__('uuid').uuid4,ScanObjectMissing=Missing,
            RecoverableCycleError=RuntimeError,MoveToReady=NS(Goal=lambda:None))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'execute','exec'),scope)
        result=asyncio.run(scope['execute'](node,NS(request=NS(target_id=''),is_cancel_requested=False)))
        self.assertTrue(result.success)
        self.assertEqual(calls,['cycle','ScanEmptyReturnReady',('hold',False),'ready','cycle'])
        self.assertEqual(node.repeat_failed,1)
        self.assertEqual(node.repeat_completed,1)


if __name__=='__main__':unittest.main()
