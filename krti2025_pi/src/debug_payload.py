#!/usr/bin/env python3

import rospy
from krti2024_pi.msg import DResult
from krti2024_pi.srv import Activate, ActivateRequest

class PayloadDebugger:
    def __init__(self):
        rospy.init_node("payload_debugger")
        
        # Subscribe ke hasil deteksi
        self.target_sub = rospy.Subscriber(
            "/vision/target/result", DResult, self.target_callback
        )
        
        # Service proxy untuk aktivasi
        self.activate_target = rospy.ServiceProxy("/vision/activate/target", Activate)
        
        self.detection_count = 0
        self.last_detection_time = rospy.Time.now()
        
    def target_callback(self, msg):
        if msg.is_found:
            self.detection_count += 1
            self.last_detection_time = rospy.Time.now()
            rospy.loginfo(f"🎯 PAYLOAD TERDETEKSI #{self.detection_count}")
            rospy.loginfo(f"   dx: {msg.dx:4d} pixels, dy: {msg.dy:4d} pixels")
            rospy.loginfo(f"   x_m: {msg.dx_m:6.3f} m, y_m: {msg.dy_m:6.3f} m")
            rospy.loginfo(f"   Distance from center: {(msg.dx**2 + msg.dy**2)**0.5:.1f} pixels")
            
            # Status alignment
            tolerance = 25
            x_aligned = abs(msg.dx) <= tolerance
            y_aligned = abs(msg.dy) <= tolerance
            
            status = "✅ ALIGNED" if (x_aligned and y_aligned) else "❌ NOT ALIGNED"
            rospy.loginfo(f"   Alignment status: {status}")
            
        else:
            # Log setiap 2 detik jika tidak ada deteksi
            if rospy.Time.now() - self.last_detection_time > rospy.Duration(2.0):
                rospy.logwarn("⚠️  Payload tidak terdeteksi - periksa:")
                rospy.logwarn("   - Posisi drone di atas target")
                rospy.logwarn("   - Pencahayaan dan kontras warna")
                rospy.logwarn("   - Parameter HSV di launch file")
                self.last_detection_time = rospy.Time.now()
    
    def activate_detection(self, target_type=1):
        """Aktivasi deteksi target"""
        try:
            rospy.loginfo(f"🔍 Mengaktifkan deteksi target type {target_type}...")
            response = self.activate_target(ActivateRequest(True, target_type))
            if response.result:
                rospy.loginfo("✅ Deteksi target berhasil diaktifkan")
            else:
                rospy.logerr("❌ Gagal mengaktifkan deteksi target")
        except Exception as e:
            rospy.logerr(f"❌ Error aktivasi: {e}")
    
    def run(self):
        rospy.loginfo("🚁 Payload Debugger Started")
        rospy.loginfo("📋 Monitoring payload detection...")
        
        # Aktivasi deteksi target 1 secara otomatis
        rospy.sleep(1.0)
        self.activate_detection(1)
        
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Status summary setiap 10 detik
            if self.detection_count > 0 and rospy.Time.now().to_sec() % 10 < 1:
                rospy.loginfo(f"📊 Total deteksi: {self.detection_count}")
            
            rate.sleep()

if __name__ == "__main__":
    try:
        debugger = PayloadDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass