/**
 * @file vector2d.h
 * @brief 定义一个简单的2维向量的类: (x,y)
 *  定义了以下运算：
 *        + 加法
 *        - 减少
 *        * 乘法
 *        / 除法
 *        - 负号
 *  定义了以下标准输出：
 *       <<
 *  定义以下函数：
 *    dot: 点乘
 *    length()： 欧氏长度
 *    sqlength()： 平方长度
 *    ort(b): 返回对对一个向量b的正交补，即将该向量向另一向量b和垂直方向b'投影，返回于向一向量的那部分(b')
 */

#ifndef VECTOR2D
#define VECTOR2D

#include <iostream>
namespace HybridAStar {
//###################################################
//                                            VECTOR2
//###################################################
/// A class describing a simple 2D vector
class Vector2D {
 public:
  /// default constructor
  inline Vector2D(const float x = 0, const float y = 0) { this->x = x; this->y = y; }
  /// a method to multiply a vector by a scalar
  inline Vector2D operator * (const float k) const { return Vector2D(x * k, y * k); }
  /// a method to divide a vector by a scalar
  inline Vector2D operator / (const float k) const { return Vector2D(x / k, y / k); }
  /// a method to add a vector to a vector
  inline Vector2D operator + (const Vector2D& b) const { return Vector2D(x + b.x, y + b.y); }
  /// a method to subtract a vector from a vector
  inline Vector2D operator - (const Vector2D& b) const { return Vector2D(x - b.x, y - b.y); }
  /// a method to negate a vector
  inline Vector2D operator - () const  {return Vector2D(-x, -y);}
  /// a convenience method to print a vector
  friend std::ostream& operator<<(std::ostream& os, const Vector2D& b) {os << "(" << b.x << "|" << b.y << ")"; return os; }
  /// a method to calculate the length of the vector
  float length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
  /// a method to calculate the length of the vector
  float sqlength() const { return x*x + y*y; }
  /// a method to calculate the dot product of two vectors
  float dot(Vector2D b) { return x * b.x + y * b.y; }
  ///a method that returns the orthogonal complement of two vectors
  inline Vector2D ort(Vector2D b) {
    Vector2D a(this->x, this->y);
    Vector2D c;
    // multiply b by the dot product of this and b then divide it by b's length
    c = a - b * a.dot(b) / b.sqlength();
    return c;
  }
  inline float getX() { return x; }
  inline float getY() { return y; }
  //  void setT(float t) { this->t = t; }
  //  float getT() { return t; }
 private:
  /// the x part of the vector
  float x;
  /// the y part of the vector
  float y;
  //  /// the theta part for plotting purposes
  //  float t;
};
inline Vector2D operator * (double k, const Vector2D& b) {
  return (b * k);
}
}
#endif // VECTOR2D
