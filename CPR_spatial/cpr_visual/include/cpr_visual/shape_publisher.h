/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef SHAPE_PUBLISHER_H
#define SHAPE_PUBLISHER_H

#include "cpr_visual/zmq.hpp"
#include "cpr_visual/shape_msg.h"



namespace cpr_visual
{

class ShapePublisher
{
public:
  ShapePublisher(std::chrono::milliseconds timeout = std::chrono::milliseconds{10});
  ~ShapePublisher();

  void publish(const ShapeMsg &msg);

private:
  zmq::context_t ctx;
  zmq::socket_t sock;
  std::chrono::milliseconds timeout;
};

}  //  END namespace cpr_visual

#endif // SHAPE_PUBLISHER_H
