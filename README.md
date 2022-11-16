# Giới thiệu
Đây là trang mã nguồn phần mềm cho xe tự hành Beetle.
# Thông tin tra cứu
- Mô hình mô phỏng của xe và nhiễu cảm biến nằm trong tập tin beetle_description/urdf/beetle.urdf
- Tất cả thông số cài đặt của hệ nằm trong tập tin beetle_navigation2/config/navigation_params.yaml
- Cây hành vi dạng xml được lưu trong beetle_navigation2/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
# Các gói phần mềm
- ackermann_drive_controller - mã nguồn bộ điều khiển động học cho xe kiểu Ackermann
- beetle_description - chứa mô hình mô phỏng xe
- beetle_gazebo - mô phỏng và bộ điều khiển động học
- beetle_navigation2 - hệ thống phần mềm xe và thông số
- beetle_evaluation - tự động hóa việc chạy và ghi chép thí nghiệm
# Kết quả thí nghiệm
