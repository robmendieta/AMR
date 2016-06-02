/* -*- mode: c++; -*- */

/** @file map_store_beam.h Interface for a beam (line) class.
 *
 *  The beam class allows to trace a beam (or line) through some grid.
 */

#ifndef MAP_STORE_BEAM_H
#define MAP_STORE_BEAM_H

#include <math.h>

namespace mapstore {

  /** Represet a beam or trace in a map.
   *
   *  This class represents a beam starting from a point in a map and
   *  going into some direction.  It computes one-after-one all cells
   *  touched by the beam.  For this the map is assumed to be made of
   *  grid cells with egde length of one (i.e. unit cells).  The
   *  represented beam does not need to start at a cell's center
   *  (i.e. start coordinates do not need to be integers).  For
   *  non-integer start coordinate, the class will correctly identify
   *  which cells are touched and which not.
   */
  class MapStoreBeam {
  public:

    /** Initialize a beam. 
     *
     *  This methods initializes the beam class.  A beam is given by its
     *  start position (which does not need to be on the grid, ie. the
     *  map is interpreted as being made of cell of finite size), its
     *  direction and an optional max length.  If length is zero, it is
     *  interpreted as unlimited.
     *
     *  The start position is given in map coordinates.
     *
     *  The direction @a a_dir is given in radians, the length is given
     *  in units of cells.
     *
     */
    MapStoreBeam(double x, double y, double dir_a, double length);

    /** Initialize beam based on start and end point.
     *
     *  The start and end point are given in map coordinates.  It is
     *  valid to set both points to the same value, in this case a line
     *  of one point length will be computed, and nextCell() will return
     *  this point and indicate end of line at its first call by setting
     *  the return value to false.
     */
    MapStoreBeam(int x1, int y1, int x2, int y2);

    /** Re-initialize an existing MapStoreBeam to start over or do another trace.
     *
     *  For the meaning of the parameters refer to the constructor
     *  documentation.
     */
    void reInit(int x1, int y1, int x2, int y2);

    /** Return map coordinates of first cell of beam. */
    void getInitCell(int &x, int &y) const {
      x = initCellX;
      y = initCellY;
    }

    /** Return map coordinates of the latest computed cell of the beam.
     *
     *  This function again returns the cell coordinates computed and
     *  returned by the last call to the nextCell() function.  See there
     *  for details.
     */
    bool getLastCell(int &x, int &y) const {
      makeMapCoord(x,y);
      return (unlimited || (lastCellX <= endX && lastCellY <= endY));
    }

    /** Return the last cell which belongs to the beam.
     *
     *  This function returns the last cell belonging to the beam,
     *  independent of if the beam has already reached that cell or not.
     *  The cell corrdinates are stored in the supplied variables @a x
     *  and @a y.
     *
     *  This function's result is undefined, if the MapStoreBeam was
     *  constructed with unlimited length.
     */
    void getEndCell(int &x, int &y) const {
      makeMapCoord(x,y, endX, endY);
    }

    /** Return the beam length traveled so far.
     *
     *  This function returns the Euclidian distance from the startpoint
     *  of the beam to the last cell returned by nextCell().
     */
    double getLen(void) const {
      return sqrt(lastCellX*lastCellX + lastCellY*lastCellY);
    }

    /** Return next cell in beam.
     *
     *  This method computes based on internal state the next cell
     *  touched by the beam.  The cell coordinates returned in the
     *  parameters x and y are in map coordinates.
     *
     *  @return The methods return value is set to false, if a non-zero
     *  length has been passed to the class' constructor and the beam
     *  has exceeded this length.  Otherwise true is returned.
     *
     *  @retval true : (1) no length was specified with the dir,len
     *  taking constructor.  (2) A length was specified or a constructor
     *  which takes an end point has been used and the cell coordinates
     *  returned in parameters x and y are within the requested beam
     *  length.
     *
     *  @retval false : A length was specified or a constructor which
     *  takes an end point has been used and the cell coordinates
     *  returned in parameters x and y exceed the specified beam length
     *  or end point.
     */
    bool nextCell(int &x, int &y);

  private:
    /** Disabled default constructor. */
    MapStoreBeam() : maxLen(0), y2Sum(0), initCellX(0), initCellY(0), lastCellX(0), lastCellY(0), xDelta(0), yDelta(0), x2Delta(0),  xHalfStep(false), yHalfStep(false) {}

    /** @internal Internal second part of constructor. */
    void init2ndStage(void);

    /** @internal Internal debug function. */
    bool nextCell2(int &x, int &y);

    /** @a internal Swap content of two integer variables. */
    void swap(int &a, int &b) const;

    /** @a internal convert internal coordinates (@a xIn, @a yIn) into
     *  map coordinates relativ to beam start cell initCellX and
     *  initCellY.  Converted coordinates are returned in @ax and @a y.
     */
    void makeMapCoord(int &x, int &y, const int xIn, const int yIn) const;

    /** @internal Return last cell in map coordinates.
     *
     *  This function converts the cell coordinates returned by the last
     *  call to nextCell() into map coordinates and returns the
     *  coordinates by setting the suplied variables @a x and @a y.
     */
    void makeMapCoord(int &x, int &y) const;


    double initPosX;
    double initPosY;

    double dir;
    double maxLen;

    int y2Sum;

    int octant;

    /** @{ start cell of the beam in map cordinates. */
    int initCellX;
    int initCellY;
    /** @} */

    /** @{ last cell cordinates computed. */
    int lastCellX;
    int lastCellY;
    /** @} */

    /** @{ End cell of the beam in map coordinates. */
    int endX;
    int endY;
    /** @} */

  
    int xDelta;
    int yDelta;
    int x2Delta;

    bool xHalfStep;
    bool yHalfStep;

    bool unlimited;
  };

}  // closes namespace mapstore

#endif // MAP_STORE_BEAM_H
