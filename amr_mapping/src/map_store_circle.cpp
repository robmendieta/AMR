/* -*- mode: c++; -*- */

/** @file map_store_circle.cpp Implement an iterator over a circle. */

#include <cmath>

#include "map_store_circle.h"

namespace mapstore {

MapStoreCircle::MapStoreCircle(double x, double y, double radius)
: origin_x(x)
, origin_y(y)
{
  // Only examine bottom half of quadrant 1, then apply symmetry 8 ways
  for (Point pt(0, 0); pt.x <= radius; pt.x++, pt.y = 0)
  {
    for (; pt.y <= pt.x && pt.square() <= radius * radius; pt.y++)
    {
      points.insert(pt);
      points.insert(Point(-pt.x, pt.y));
      points.insert(Point(pt.x, -pt.y));
      points.insert(Point(-pt.x, -pt.y));
      points.insert(Point(pt.y, pt.x));
      points.insert(Point(-pt.y, pt.x));
      points.insert(Point(pt.y, -pt.x));
      points.insert(Point(-pt.y, -pt.x));
    }
  }
  iter = points.begin();
}

bool MapStoreCircle::nextCell(int &x, int &y)
{
  if (iter != points.end())
  {
    x = lround(origin_x + iter->x);
    y = lround(origin_y + iter->y);
    iter++;
    return true;
  }
  else
  {
    return false;
  }
}

bool MapStoreCircle::Point::operator<(const Point& pt) const
{
  if (square() < pt.square())
    return true;
  if (pt.square() < square())
    return false;
  if (x < pt.x)
    return true;
  if (pt.x < x)
    return false;
  return y < pt.y;
}

}

// End
