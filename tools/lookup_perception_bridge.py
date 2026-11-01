"""Explicit perception lease ownership: lookup acquisition versus BT barcode scan."""
import rclpy
from std_srvs.srv import SetBool
from tracking_icp_monitor import Monitor


class LookupPerceptionBridge(Monitor):
    def __init__(self):
        self.held=False
        super().__init__()
        self.create_service(SetBool,'/task/lookup_perception_hold',self.set_hold)

    def set_hold(self,request,response):
        changed=self.held!=request.data
        self.held=request.data
        if changed and not self.held:
            self.target='';self.last_target='';self.req=None
            self.lease('detect','')
        response.success=True
        response.message='BT owns barcode leases' if self.held else 'lookup acquisition owns leases'
        return response

    def lease(self,mode,target):
        if not self.held:super().lease(mode,target)

    def detections(self,msg):
        if not self.held:super().detections(msg)

    def objects(self,msg):
        if not self.held:super().objects(msg)

    def tracking_event(self,msg):
        if not self.held:super().tracking_event(msg)

    def tick(self):
        if not self.held:super().tick()


def main():
    rclpy.init();node=LookupPerceptionBridge()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
