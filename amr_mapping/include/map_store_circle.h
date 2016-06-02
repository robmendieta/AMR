/* -*- mode: c++; -*- */

/** @file map_store_circle.h Interface definition for an iterator over a circle. */

#ifndef MAP_STORE_CIRCLE_H
#define MAP_STORE_CIRCLE_H

#include <set>

namespace mapstore {

  /** Implement an iterator over a circle.
   *
   *  this class implements an iterator which enumerates all cells of
   *  a grid-based map covered by a circle originating at a given
   *  cell and having a given radius. Cells are enumerated from the origin
   *  on, progressing towards the edges of the circle. While strictly
   *  increasing cell to origin distances for consecutively returned cells
   *  can not be guaranteed, cells are returned in mostly increasing
   *  distance order.
   */
  class MapStoreCircle {
  public:

    /** Construct a circle iterator.
     *
     *  This constructor makes a circle iterator for a circle
     *  located at position (@a x, @a y) in the map and having a radius
     *  @a radius.
     *
     *  @param x  x coordinate of circle center.
     *  @param y  y coordinate of circle center.
     *  @param radius radius of the circle
     */
    MapStoreCircle(double x, double y, double radius);

    /** Empty destructor */
    ~MapStoreCircle() { };

    /** Compute next cell in circle.
     *
     *  This function computes the next cell in the circle, based on
     *  internal state.  It is guaranteered that every cell covered by
     *  the circle is returned exactly once.
     *
     *  The next cell's coordinates are returned in the supplied
     *  variables @a x and @a y.  The return value of this function
     *  indicates if more cells are available (if return value is true)
     *  or if all cells covered by the cone have been returned (return
     *  value is false). No assumptions should be made regarding the
     *  order in which the cells are returned.
     *
     *  @param x  [out] x coordinate of next cell in the circle.  The
     *   cells are enumerated by an internal state.  No assumptions
     *   should be made regarding the order in which the cells are
     *   returned.
     * 
     *  @param y  [out] y coordinate of next cell in the circle.  The
     *   cells are enumerated by an internal state.  No assumptions
     *   should be made regarding the order in which the cells are
     *   returned.
     * 
     *  @return flag if more cells are to be returned.  If false, all
     *   cells in the circle have been returned.  
     */
    bool nextCell(int &x, int &y);

  private:

    struct Point
    {
      double x;
      double y;
      double square() const { return x * x + y * y; }
      Point(double x = 0, double y = 0) : x(x), y(y) { }
      bool operator<(const Point& pt) const;
    };

    std::set<Point> points;
    std::set<Point>::const_iterator iter;
    double origin_x;
    double origin_y;

  };

}  // closes namespace mapstore.

#endif // MAP_STORE_CIRCLE_H
