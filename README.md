# Thesis
## Đồ án tốt nghiệp: Nghiên cứu phát triển, điều khiển phân tán cho hệ thống đa robot di chuyển theo bầy.

Đây là thư viện của đồ án tốt nghiệp. Thuật toán được phát triển dựa trên thuật toán đổ xô theo bầy của Raynold. Ứng dụng trên các robot có các cảm biến
rời rạc.

- ### Phần mềm sử dụng: Webots, Matlab, Visual Studio.
- ### Ngôn ngữ: C++, Matlab.
- ### Robot: Mô hình robot hai bánh vi sai, robot e-puck, robot trong bị động cơ di chuyển, và chỉ sử dụng hai loại cảm biến: La bàn số và Cảm biến khoảng cách.
## Mục tiêu: 
Trong robot bầy đàn, việc robot di chuyển theo một bầy có trật tự và duy trì các thành viên trong bầy vô cùng quan trọng, là bài toán đầu tiên cần phải giải quyết trong chuỗi nhiệm vụ triển khai bầy robot cho các ứng dụng.

## 1. Mô tả bài toán:
Bầy robot gồm nhiều các robot con giống nhau về mặt cấu trúc. Và vì số lượng của bầy rất lớn lên bài toán sử dụng mô hình robot với các cảm biến đơn giản để có thể tiết kiệm chi phí và đáp ứng các thuật toán bầy đàn. Nghiên cứu với mô hình robot thực thì cảm biến lân cận của robot sẽ là cảm biến rời rạc. Sử dụng các mô hình cảm biến rời rạc sẽ xuất hiện các điểm mù trong vùng giới hạn nhìn thấy của robot. Bài toán cần giải quyết được đưa ra làm sao để bầy robot có thể đảm bảo di chuyển về mặt các thành viên trong bầy. 
Mô tả robot với cảm biến rời rạc là các tia như hình ![Image_0](src/Distributed.png| width=100)

## 2. Thuật toán áp dụng:
Áp dụng mô hình thuật toán bầy đàn của Reynold, gồm 3 hành vi điều khiển các thành viên trong bầy:






