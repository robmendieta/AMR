/* -*- mode: c++; -*- */

/** @file map_store_cone.h Interface definition for an iterator over a circular sector. */

#ifndef MAP_STORE_CONE_H
#define MAP_STORE_CONE_H

#include "map_store_beam.h"

namespace mapstore {

  /** Implement an iterator over a sonar cone.
   *
   *  this class implements an iterator which enumerates all cells of
   *  a grid-based map covered by a sonar cone originating at a given
   *  cell and looking in a given direction.  Cells are enumerated
   *  from the sonar sensor on, progressing towaerds the scan limit of
   *  the sensor.  While strictly increasing cell to origin distances
   *  for consecutively returned cells can not be guaranteed, cells
   *  are returned in mostly increasing distance order.
   */
  class MapStoreCone {
  public:

    /** Construct a sonar cone iterator.
     *
     *  This constructor makes a sonar cone iterator for a sonar sensor
     *  located at position (@a x, @a y) in the map and looking into
     *  direction @a dir width a cone width of @a width and a maximum
     *  sensing range of @a len.
     *
     *  the direction @a dir is measured in radians against the x axis
     *  of the maps coordinate system.  The scan range @a len is
     *  measured in units of cells, i.i cell edge length.
     * 
     *  @param x  x coordinate of sensor position.
     *  @param y  y coordinate of sensor position.
     *  @param dir  orientation of sensor, i.e. direction of center beam.
     *  @param width sonar cone opening angle in radians.
     *  @param len   maximum sensing distance of the sonar sensor.
     */
    MapStoreCone(double x, double y, double dir, double width, double len);

	/** Default Destructor, frees resources allocated to maintain internal state. */
    ~MapStoreCone();

    /** Compute next cell in cone.
     *
     *  This function computes the next cell in the sonar cone, based on
     *  internal state.  It is guaranteered that every cell covered by
     *  the sonar cone is returned exactly once.
     *
     *  The next cell's coordinates are returned in the supplied
     *  variables @a x and @a y.  The return value of this function
     *  indicates if more cells are available (if return value is true)
     *  or if all cells covered by the cone have been returned (return
     *  value is false).  No assumptions should be made regarding the
     *  order in which the cells are returned.
     * 
     *  Example:
     *  @code
     *  MapStore map(size_X, size_Y);
     *  MapStoreCone msc(sensor_x, sensor_y, sensor_orient);
     * 
     *  int x=0, y=0;
     *  while (msc.nextCell(x,y)) {
     *      // do something with the cell
     * 		double mapVal = map.get(x,y);
     *  }
     *  @endcode
     * 
     *  @param x  [out] x coordinate of next cell in the cone.  The
     *   cells are enumerated by an internal state.  No assumptions
     *   should be made regarding the order in which the cells are
     *   returned.
     * 
     *  @param y  [out] y coordinate of next cell in the cone.  The
     *   cells are enumerated by an internal state.  No assumptions
     *   should be made regarding the order in which the cells are
     *   returned.
     * 
     *  @return flag if more cells are to be returned.  If false, all
     *   cells in the cone have been returned.  
     */
    bool nextCell(int &x, int &y);

  private:

    /** @internal update cell cache.  Used to remove dublicates from
	result set. */
    bool markDone(int x, int y);

    /** @internal @{ Cone borders and center beam.
     *
     *  These three beams define the cone's direction and borders.
     */
    MapStoreBeam beamInfoCenter;
    MapStoreBeam beamInfoLeft;
    MapStoreBeam beamInfoRight;
    /** @} */

    /** @internal @{ Curent position in cone.
     *
     *  The cone is processed sphere by sphere starting with the its
     *  origin.  These variables define the current sphere.
     *  @}
     */
    int xIdxCenter,yIdxCenter;
    int xIdxLeft,yIdxLeft;
    int xIdxRight,yIdxRight;

    MapStoreBeam beamInfoLeftFill;
    MapStoreBeam beamInfoRightFill;

    int state;
    bool leftInvalid;
    bool rightInvalid;

    int minX, minY;
    int sizeX, sizeY, sizeXRow;

    /** @internal @{  cell cache, used to remove dublicates from result set. */
    int cachehit, cachemiss;

    int *cache;
    /** @} */

  };

}  // closes namespace mapstore.

#endif // MAP_STORE_CONE_H
