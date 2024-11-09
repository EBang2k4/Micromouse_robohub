#include <Dynamixel2Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <VL53L0X.h>

#define RX_PIN 13  // Chân Rx của OpenRB-150 (kết nối với Tx của ESP32)
#define TX_PIN 14  // Chân Tx của OpenRB-150 (kết nối với Rx của ESP32)

#if defined(ARDUINO_OpenRB)  // Khi sử dụng OpenRB-150
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;  // Không cần DIR control pin
#endif

float Kp = 0.56; // Hệ số tỉ lệ
float desiredDistanceLeft = 60; // Khoảng cách mong muốn bên trái (mm)
float previousErrorLeft = 0;

VL53L0X sensor1; //trai
VL53L0X sensor2; //phai
Dynamixel2Arduino  dxl(DXL_SERIAL, DXL_DIR_PIN);
const float DXL_PROTOCOL_VERSION = 2.0;
#define XSHUT1 3
#define XSHUT2 2

using namespace ControlTableItem;

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

int maze[MAZE_WIDTH][MAZE_HEIGHT]; // 0 là ô chưa đi qua, 1 là ô đã đi qua
int visitedCells = 0; // Biến đếm số ô đã đi qua
int path[100][2]; // Mảng lưu đường đi (tối đa 100 bước)

int currentX = 0, currentY = 0; // Vị trí bắt đầu
int destinationX = 9, destinationY = 9; // Vị trí đích (có thể thay đổi theo yêu cầu)

void setup() {
  DEBUG_SERIAL.begin(115200);
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // Đưa cả hai chân XSHUT về mức LOW để reset cảm biến
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Khởi động cảm biến 1 và gán địa chỉ mới
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(0x31); // Gán địa chỉ mới cho cảm biến 1

  // Khởi động cảm biến 2 và gán địa chỉ mới
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(0x32); // Gán địa chỉ mới cho cảm biến 2

  // Cấu hình chế độ cảm biến
  sensor1.startContinuous();
  sensor2.startContinuous();
  dxl.begin(57600);  // Tốc độ baud
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(1);
  dxl.ping(2);

  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_VELOCITY);
  dxl.torqueOn(1);

  dxl.torqueOff(2);
  dxl.setOperatingMode(2, OP_VELOCITY);
  dxl.torqueOn(2);

  for (int i = 0; i < MAZE_WIDTH; i++) {
    for (int j = 0; j < MAZE_HEIGHT; j++) {
      maze[i][j] = 0; 
    }
  }

  maze[currentX][currentY] = 1;
  visitedCells = 1;
  path[0][0] = currentX;
  path[0][1] = currentY;
}

void loop() {
  if (Serial2.available() > 0) {
    char c = Serial2.read();  // Đọc từng byte
    Serial.println(c);
  // Tuỳ chọn: Xóa bộ đệm nếu dữ liệu bị lặp lại
    while (Serial2.available() > 0) Serial2.read();
    delay(10);  // Độ trễ nhỏ để tránh đọc lặp lại

  int distanceLeft = sensor1.readRangeContinuousMillimeters();
  Serial.print("Sensor 1: ");
  Serial.print(distanceLeft);
  Serial.print("   ");  // Add space between values

  int distanceRight = sensor2.readRangeContinuousMillimeters();
  Serial.print("Sensor 2: ");
  Serial.println(distanceRight);  // Use println() to move to a new line after printing the second sensor value

    if (c == 'N' && distanceLeft > 100 && distanceRight > 100) {
      moveForward();
    } 
    else if(distanceLeft > 100 && maze[currentX-1][currentY] == 0) {
        turnLeft();
      } else if (distanceRight > 100 && maze[currentX+1][currentY] == 0) {
        turnRight();
      } else {
        turnAround();
      }
  }
}

void moveForward() {
  dxl.setGoalVelocity(1, -400);
  dxl.setGoalVelocity(2, 400);

  maze[currentX][currentY] = 1;
  visitedCells++;

  path[visitedCells][0] = currentX;
  path[visitedCells][1] = currentY;

  currentY++;
}

void turnLeft() {
  dxl.setGoalVelocity(1, 0);
  dxl.setGoalVelocity(2, 300);
  delay(500);

  currentX--;
  maze[currentX][currentY] = 1;
  visitedCells++;

  path[visitedCells][0] = currentX;
  path[visitedCells][1] = currentY;
}

void turnRight() {
  dxl.setGoalVelocity(1, -300);
  dxl.setGoalVelocity(2, 0);
  delay(500);

  currentX++;
  maze[currentX][currentY] = 1;
  visitedCells++;

  path[visitedCells][0] = currentX;
  path[visitedCells][1] = currentY;
}

void turnAround() {
  dxl.setGoalVelocity(1, 300);
  dxl.setGoalVelocity(2, -300);
  delay(1000);

  currentX = 0;
  currentY = 0;  
  maze[currentX][currentY] = 1;
  visitedCells++;

  path[visitedCells][0] = currentX;
  path[visitedCells][1] = currentY;
}

// Hàm tính toán đường đi ngắn nhất từ đích về start
void calculateShortestPath() {
  int startX = destinationX, startY = destinationY; // Vị trí bắt đầu (bây giờ là đích)
  int endX = 0, endY = 0; // Vị trí kết thúc (bây giờ là start)
  
  int dist[MAZE_WIDTH][MAZE_HEIGHT];  // Lưu trữ độ dài từ điểm bắt đầu đến các ô khác

  // Khởi tạo khoảng cách
  for (int i = 0; i < MAZE_WIDTH; i++) {
    for (int j = 0; j < MAZE_HEIGHT; j++) {
      dist[i][j] = 9999; // Giá trị lớn để chỉ các ô chưa được thăm
    }
  }

  dist[startX][startY] = 0;  // Đặt điểm xuất phát (đích cũ) có khoảng cách là 0

  // Dijkstra (hoặc A*) có thể được triển khai ở đây để tìm đường đi ngắn nhất.
  // Đảm bảo chỉ di chuyển qua những ô đã đi qua trong maze (maze[i][j] == 1).
  
  // Sau khi tính toán, lưu đường đi vào biến path.

  // Hiển thị kết quả
  Serial.println("Shortest Path:");
  for (int i = visitedCells - 1; i >= 0; i--) { // Lật ngược đường đi để từ đích về start
    Serial.print("(");
    Serial.print(path[i][0]);
    Serial.print(",");
    Serial.print(path[i][1]);
    Serial.println(")");
  }
}
