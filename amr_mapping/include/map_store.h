/* -*- mode: c++; -*- */

/** @file map_store.h
 *
 *  Implementation of the MapStore class.
 *
 *  This file implements the MapStore class, which is used to store
 *  growable grid-based maps.  Map data is stored in double values.
 */

#ifndef MAP_STORE_H
#define MAP_STORE_H

#include <list>
#include "map_store_error.h"
#include "map_store_beam.h"

namespace mapstore {

  /** Representation of one grid cell.
   *
   *  This structure represents a map cell for those occasions where
   *  individual grid cells are passed between function and classes.  It
   *  is primarily used with the MapStore::trace() function and the
   *  MapStoreTrace class returned by that function.
   *
   *  The MapStore class itself uses another, more memory-efficient
   *  internal representation.
   */
  struct MapStoreCell {
    /** Default constructor.  Not used.*/
    MapStoreCell() : x(0), y(0), val(0) {}

    /** Create new MapStoreCell for cell (@a x, @a y) with cell value
     *  @a aval. */
    MapStoreCell(int ax, int ay, double aval) : x(ax), y(ay), val(aval) {}

    int x;  /** x-coordinate of cell. */
    int y;  /** y-coordinate of cell. */
    double val;  /** The value stored in this cell. */
  };


  /** This class reperesents a trace (or beam) through a map.
   *
   *  This class records all cells touched by a beam through a map,
   *  starting at a point (x, y) in the some direct, using instances of
   *  MapStoreCell.
   *
   *  This class' primariy use is as result of a call to
   *  MapStore::trace().
   */
  class MapStoreTrace {
  public:
    /** Default constructor, create an empty trace.*/
    MapStoreTrace();

    /** Add a cell to the trace.
     *
     *  Cells @em must be added in the correct order, i.e. in the order
     *  a beam / trace would touche the cells!
     *
     *  The supplied MapStoreCell instance is copied and can be reused
     *  to add another cell or can be destroyed.
     */
    void add(const MapStoreCell &c);

    /** Add a cell to the trace.
     *
     *  Cells @em must be added in the correct order, i.e. in the order
     *  a beam / trace would touche the cells!
     */
    void add(int x, int y, double val);

    /** Public iterator type to walk through the trace.
     *
     *  This iterator allows one to access all cells in the trace one by
     *  one.
     *  @code
     *  MapStore mapstore;
     *  // ... do something with the map store ...
     *
     *  // obtain a trace
     *  MapStoreTrace mst = mapstore.trace(...);
     *
     *  // walk the trace
     *  MapIt myIterator = mst.traceStart();
     *  MapIt myEnd = mst.traceEnd();
     *  while (myIterator != myEnd) {
     *     MapStoreCell msc = *myIterator;
     *     myIterator++;
     *     // ... do something with the cell ...
     *     printf("cell (%d, %d) has value %f\n", msc.x, msc.y, msc.val);
     *  }
     *  @endcode
     *
     *  Not to be confused with MapStore::MapIt.
     */
    typedef std::list<MapStoreCell>::iterator MapIt;

    /** Returns an iterator pointing to the beginning of the trace.
     *
     *  See MapStoreTrace::MapIt for details.
     */
    MapIt traceStart(void) {
      return cells.begin();
    }

    /** Returns an iterator pointing to the end of the trace.
     *
     *  See MapStoreTrace::MapIt for details.
     */
    MapIt traceEnd(void) {
      return cells.end();
    }

  private:
    /** We use a standard lib "list" as storage backend. */
    std::list<MapStoreCell> cells;
  };


  /** This class implements a storage for a grid-based map.
   *
   *  Initially the map is zero-centered, that is the coordinates (0,0)
   *  are in its center.  However the map can be grown at arbitrary
   *  places, what may shift the map's center away from (0,0).
   *
   *  The grid cells are addressed by integer numbers.  Conceptionally
   *  each cell (x,y) occupies the space from (x-0.5, y-0.5) exclusive
   *  to (x+0.5, y+0.5) inclusive.  However this comes only into
   *  importance, when tracing a beam through the map to identifiy which
   *  cells are touched by the beam.
   *
   *  The class does not make any assumption about the unit size of the
   *  grid cells.  Cells are simple numbered by integer coordinates.
   *  See class SonarMap which does care about unit size.
   */
  class MapStore {
  public:
    /** Constructor to create empty map of given size.
     *
     *  Creates a map of the requested size with zero-centred origin.
     *  The map includes all coordiante points from -sizeX/2 to sizeX/2
     *  in x and -sizeY/2 to sizeY/2 y direction.  If sizeX or sizeY is
     *  odd, it will be rounded down first.
     *
     *  @param initSizeX Initial size in x direction.  The map's center
     *  coordinate will be 0,0, with the map extending initSizeX/2
     *  cells in both directions.
     *
     *  @param initSizeY Initial size in y direction.  The map's center
     *  coordinate will be 0,0, with the map extending initSzeX/2
     *  cells in both directions.
     */
    MapStore(int initSizeX, int initSizeY);

	/** Constructor to load map from file.
	 * 
	 *  Cretes a new map object from the map data stored in the named
	 *  file.  See loadMap() for a documentation of the file format.
	 *  This constructor throws an exception of type
	 *  MapStoreError::Format in case of unreadable file or file format
	 *  error.
	 * */
	MapStore(const char *mapFileName);

    /** Destructor of this class. */
    ~MapStore();
    
    /** Load a map from the named file.
     * 
     *  This function loads the maps content from the named file.  See
     *  loadMap(FILE *fh) for more details.
     * 
     *  @p Map file format
     *  The map file format is line-based.  The file starts with a
     *  header describing the size and possibly resolution of the map. 
     *  The resolution is not used by MapStore.  The header is
     *  recognized by the keywords defining map properties starting in
     *  the first column of a line.  The header must be present as one
     *  continuous sequence of lines without empty lines in between.
     * 
     *  The header may be followed by additional lines holding
     *  information on how to print the map using the GNUPlot data
     *  visualizer.  These lines are identified by starting with a
     *  lower-case letter and not holding as first word one of the map
     *  property keywords.
     * 
     *  The optional visualization information is followed by the raw
     *  map data.  The map data is stored as (long) lines of decimal
     *  numbers, each row representing one row of the map and each
     *  column representing one column of the map.
     * 
     *  @p Map header
     *  The map header can hold the following keyword in arbitrary order:
     *  * MapSize: columns rows
     *  * MapRes: cell_size_in_meters
     *  * MapType: type_name
     *  * MapOrigin: center_x center_y
     * 
     * 
     *  @return flag if the load was successful or not.
     * */
    bool loadMap(const char *mapFileName);
    
	/** Load map from alredy open file handle.
	 * 
	 *  This function loads the map data stored in the open file fh
	 *  into the map.  It checks if the file starts at its current read
	 *  position with a map header or with raw map data.  If a map
	 *  header is found, then its information is used to reconfigure
	 *  the map (i.e. resize) as specified in the header.  If no header
	 *  is found, then the map data is read into the existing map.  It
	 *  is a fatal error, if the raw data does not fit the maps current
	 *  configuration.
	 * 
	 *  In case of unrecoverable error, i.e. the function already
	 *  modified the map and then discovered a format error in the data
	 *  file, an exception of type MapStoreError::Format is thrown.
	 *
	 *  See loadMap(const char *mapfileName) for a documentation of the
	 *  required map file format.
	 * 
	 *  @retval true : This function returns true if the map could
	 *     successfully loaded.
	 *  @retval false : This function returns false if the map could
	 *     not be loaded, for example because no valid data was found,
	 *     and the map is unaltered.  If a problem is found after the
	 *     function already modified the map, then an exception of type
	 *     MapStoreError::Format is thrown.
	 * */
	bool loadMap(FILE *mapFH);


    /** Add space to map.
     *
     *  This method adds the quadratic area around (x,y) with size @a size
     *  to the map.  These are the coordinates (inclusive) from x-size
     *  to x+size and y-size to y+size.
     *
     *  @param x Center coordinate where to add space.  Note that this
     *  coordinates does not need to be within the already defined
     *  part of the map.  The map can be grown everywhere.
     *
     *  @param y Center coordinate where to add space.  Note that this
     *  coordinates does not need to be within the already defined
     *  part of the map.  The map can be grown everywhere.
     *
     *  @param size amount of space to add into both directions around
     *  cell (@a x, @ y).  Example: grow(2, 3, 4) will add the
     *  rectagular space covering the coordinates from (-2, -1) to (6,
     *  7).
     */
    bool grow(int x, int y, int size);

    /** Set map point to given value.
     *
     *  Will throw a MapStoreError exception if (x,y) is outside the
     *  map.
     */
    void set(int x, int y, double val);

    /** Get map point value.
     *
     *  Will throw a MapStoreError exception if (x,y) is outside the
     *  map.
     */
    double get(int x, int y) const;

    /** Get a constant pointer to the internal data storage. */
    const double* getRawData() const
    {
      return data;
    }

    /** Trace a beam through the map and report all map points visited. 
     *
     *  This method trace a beam through the map.  Opposed to the set()
     *  and get() methods it takes double values as coordinates and a
     *  direction (measured in radians).  While the map works on a
     *  discrete, integer-based grid, this method takes real numbers to
     *  be able to accurately trace the beam.  It computes which grid
     *  cells are touched and reports the integer coordinates of the
     *  touched cells along with that cells value.  Each cell with
     *  integer coordinates (x,y) occupies the space from x,y -0.5
     *  exclusive to x,y + 0.5 inclusive.
     *
     *  The touched cells are returned as a MapStoreTrace object.
     *
     *  Search for cells stops if the given length limit has been
     *  reached or the beam leaves the defined map area.  If no length
     *  has been specified, or length is zero, then the search stops at
     *  the first cell with a value bigger than 0.  This cell is part of
     *  the result set returned.
     *
     *  Will throw a MapStoreError exception if (x,y) is outside the map.
     *
     *  @param x X start coordinate of beam.  Note that x can be a
     *  double, not just an integer.  The integer cell (@c a, @c b)
     *  covers the space from (@c a - 0.5, @c b - 0.5) to (@c a + 0.5,
     *  @c b + 0.5).
     *
     *  @param y See parameter @a x.
     *
     *  @param dir direction of beam, in radians measured against
     *  map's x axis.
     *
     *  @param length Length ofbem in units of cell egde length.
     */
    MapStoreTrace trace(double x, double y, double dir, double length) const;

    /** Find max value along a beam.
     *
     *  This method is similar to trace(), except that this method
     *  returns only the cell with the maximum value in the beam.
     *
     *  @sa trace(double x, double y, double dir, double length)
     */
    MapStoreCell traceMax(double x, double y, double dir, double length);

    /** Find max value in a cone. 
     *
     *  This method is similar to trace(), except it returns the cell
     *  with maximum value inside a cone of angular width @a width
     *  around the beam from (x,y) into direction dir with length len.
     *
     *  @sa trace(double x, double y, double dir, double length)
     */
    MapStoreCell traceConeMax(double x, double y, double dir, double width, double len);

    /** Set a rectangular region to value.
     *
     *  Will throw a MapStoreError exception if the whole rectangle is
     *  outside the map.
     *
     *  @param x,y  corrdinates of uper-left corner of rectangle.
     *
     *  @param @adx,dy number of cells to fill in x and y direction.
     *  If dx or dy are negativ, then the rectangle is filled to the
     *  lft or top of (x, y), respectively. (I.e dx < 0 and dy < 0
     *  makes (x, y) the lower right corner of the filled rectangle.
     *  Neither dx nor dy may be zero.
     */
    void fillRect(int x, int y, int dx, int dy, double val);

    /** Set a cone-shaped region to value.
     *
     *  Will throw a MapStoreError exception if the whole cone is
     *  outside the map.
     *
     *  @param x,y  Origin of the cone.
     *
     *  @param dir Direction of the center beam of the cone.
     *
     *  @param len Length (radius) of the cone, in units of cell egde
     *  length.
     */
    void fillCone(double x, double y, double dir, double width, double len, double val);

    /** Declare rectangular region as undefined.
     *
     *  Equivalent to calling fillRect with a zero value, excpet that
     *  this function may free some storage space for some storage
     *  engines.
     */
    void eraseRect(int x, int y, int dx, int dy);

    /** Return minimum cell coordinate in x direction. */
    int minX(void) const {
      return -originX;
    }

    /** Return map size in x direction. */
    int getSizeX(void) const {
      return sizeX;
    }

    /** Return map size in y direction. */
    int getSizeY(void) const {
      return sizeY;
    }

    /** Return map resolution. */
    double getResolution(void) const {
      return resolution;
    }

    /** Return minimum cell coordinate in y direction. */
    int minY(void) const {
      return -originY;
    }

    /** Return maximum cell coordinate in x direction. */
    int maxX(void) const {
      return sizeX-originX-1;
    }

    /** Return maximun cell coordinate in y direction. */
    int maxY(void) const {
      return sizeY-originY-1;
    }

    /** Check if @a x is a in its valid range. 
     *
     *  The value @a x is taken as the x-value of a cell coordinate
     */
    bool isInX(int x) const {
      // -originX <= x < sizeX-originX
      // 0 <= (x+originX) < sizeX
      x += originX;
      return ((0 <= x) && (x < sizeX));
    }

    /** Check if @a y is a in its valid range. 
     *
     *  The value @a y is taken as the y-value of a cell coordinate
     */
    bool isInY(int y) const {
      // -originY <= y < sizeY-originY
      // 0 <= (y+originY) < sizeY
      y += originY;
      return ((0 <= y) && (y < sizeY));
    }


    /** Special iterator to walk through a Mapstore.
     *
     *  For details, refere the the constructor documentation.
     *
     *  This class is not to be confused with MapStoreTrace::MapIt.
     */
    struct MapIt {

      /** Construct beam iterator through a MapStore.
       *
       *  This convenient structure encapsulates an instance of the
       *  MapStoreBeam class to trace a beam from cell (@a x1, @a y1) to
       *  cell (@a x2, @a y2).  For allowed values see the documentation
       *  of class MapStoreBeam.
       *
       *  The iterator is created for the map @a a_map and has a
       *  dependency to the map @a a_map.  The map may not be
       *  destroyed while this iterator exists.  Modifying the map
       *  (including growing it) is fine.
       *
       *  @param x1,y1 Start cells of beam.  The beam originates at
       *  the center of the named cell.
       *
       *  @param x2,y2 End cells of beam.  The beam terminates at the
       *  center of the named cell.
       *
       */
      MapIt(int x1, int y1, int x2, int y2, const MapStore &a_map) : beam(x1,y1, x2,y2), map(a_map) {
      }

      /** Return next cell in the trace.
       *
       *  this function determines the next cell in the trace and
       *  updates the functions parameters to refelct the next cell.  It
       *  returns "true" if there was a next cell, and "false" if the
       *  end of the trace had been reached.  In case of returning
       *  "false", the values of the parameters are undefined.
       *
       *  @param x Return parameter, the functions set @a x to the
       *  x-coordinate of the next cell.
       *
       *  @param y Return parameter, the functions set @a y to the
       *  y-coordinate of the next cell.
       *
       *  @param val Return parameter, the function sewt @a val to the
       *  next cell's value.
       *
       *  @retval true @a x, @a y and @a val have been updated.
       *
       *  @retval false The last cell has been reached (in the
       *  previous call).  The return parameters are undefined.
       */
      bool next(int &x, int &y, double &val) {
	bool res = beam.nextCell(x,y);
	if (res) {
	  val = map.get(x,y);
	}
	return res;
      }

      /** The underlying beam instance.
       *
       *  This MapStoreBeam class implements the actual computation of
       *  the next cell.
       */
      MapStoreBeam beam;

      /** Reference to the map for which this iterator has been created.*/
      const MapStore &map;
    };

    /** Create a map iterator.
     *
     *  This function creates an iterator travering the map from cell
     *  (@a x1, @a y1) to cell (@a x2, @a y2).  For allowed values see
     *  the documentation of class MapStoreBeam.  It checks if the
     *  supplied coordinates are within the map and throws an exception
     *  of type MapStoreError::Range if not.
     *
     *  The created iterator has a dependency to this map.  The map
     *  may not be destroyed while this iterator exists.  Modifying
     *  the map (including growing it) is fine.
     *
     *  @param x1,y1 Start cells of beam.  The beam originates at the
     *  center of the named cell.
     *
     *  @param x2,y2 End cells of beam.  The beam terminates at the
     *  center of the named cell.
     *
     */
    MapIt it(int x1, int y1, int x2, int y2) {
      if ( isInX(x1) && isInX(x2) && isInY(y1) && isInY(y2) ) {
	MapIt mi(x1, y1, x2, y2, *this);
	return mi;
      } else
	throw MapStoreError(MapStoreError::Range);
    }

    /** Write the map content to the file @a fh in GnuPlot format.
     * 
     *  @param fh An open C-StdLib file handle.  This can also be a
     *   process handle created using the Posix popen() function.
     *
     *  @param title The title text to use in the gnuplot drawing.
     * 
     *  @sa startGPMultiplot(), commitGPMultiplot().
     */
    void dumpGP(FILE *fh, const char *title) const;

    /** Setup multiple plots on one GNUPlot page.
     * 
     *  This function sends GNUPlot commands to @a fh to configure
     *  GNUPlot for drawing up to @a numMaps maps in a grid on one
     *  sheet. 
     * 
     *  @param fh An open C-StdLib file handle.  This can also be a
     *   process handle created using the Posix popen() function.
     *
     *  @param numMaps  Number of maps to plot.  Must be 2..4.
     * 
     *  @sa commitGPMultiplot()
     */
	static void startGPMultiplot(FILE *fh, int numMaps);
	
    /** Commit a series of at most n plots as specified with startGPMultiplot.
     * 
     *  This function ends a series of map plots which are drawn on the
     *  same sheet.  At most as many plots as specified by the numMaps
     *  parameter of startGPMultiplot maps can be commited.  If more
     *  maps have been send to the file @a fh, then gnulpot may abort. 
     *  Sending less maps then announced with startGPMultiplot is fine.
     * 
     *  Example:
     *  @code
     *  MapStore map1(size_X, sizeY), map2(size_X, sizeY), map3(size_X, sizeY);
     * 
     *  FILE *myFH = fopen("mapdata.txt","w");
     * 
     *  while(run_loop) {
     *      // process sensor data
     * 
     *      // update map
     * 
     *      // decide if we should log the map
     * 	    if (time_to_dump_map) { 
     *          MapStore::startGPMultiplot(myFH, 3);
     *          map1.dumpGP(myFH, "first map");
     *          map2.dumpGP(myFH, "next map");
     *          map3.dumpGP(myFH, "yet another map");
     *          MapStore::commitGPMultiplot(myFH);
     *      }
     *  }
     *  @endcode
     * 
     *  @param fh An open C-StdLib file handle.  This can also be a
     *   process handle created using the Posix popen() function.
     */
	static void commitGPMultiplot(FILE *fh);
	
	/** Create GNUPlot child process.
	 * 
	 *  This function creates a child process running GNUPlot, which
	 *  can be used as a file for writing map data.  Use in conjunction
	 *  with dumpGP() and/or startGPMultiplot().
	 * 
     *  Example:
     *  @code
     *  MapStore map1(size_X, sizeY);
     * 
     *  FILE *myFH = MapStore::startGP();
     * 
     *  while(run_loop) {
     *      // process sensor data
     * 
     *      // update map
     * 
     *      // decide if we should log the map
     * 	    if (time_to_dump_map) { 
     *          map1.dumpGP(myFH, "first map");
     * 
     *          // to plot multiple maps on one sheet,
     *          // see commitGPMultiplot() example.
     *      }
     *  }
     *  @endcode
     * 
	 *  @return C-StdLib file handle, opened for writing, connected to
	 *   GNUPlot.  This @em must be closed using stopGP()!  I t can
	 *   @em not be closed by simply calling fclose().
	 * 
	 */
	static FILE *startGP(void);
	
	/** Disconnect from a GNUPlot child process.
	 * 
	 *  This function closes the link to a GNUPlot child process,
	 *  causing GNUPlot to terminate.
	 * 
	 *  @param fh A file handle connected to a GNUPlot process.  This
	 *   @em must be opened by a call to startGP().
	 */
	static void stopGP(FILE *fh);
	
    /** Dump a double-array in a gnuplot compatible way.
     *
     *  This function creates a gnuplot dump from the map data
     *  starting at @a data.  The map data is assumed to be a double
     *  array of sizeX columns and sizeY rows.  The flag rowMajor
     *  (defaults to true) determines if the map dat is stored in
     *  rowMajor mode (all columns of the first row follow directly
     *  after eachother, then all columns from the second row etc.)
     *  or in column major mode.
     *
     *  @param fh An open C-StdLib file handle.
     *
     *  @param title The title text to use in the gnuplot drawing.
     *
     *  @param data Address of the double array holding the actual map
     *  data.  Note that this function assumes a flat array, that is
     *  all cells follow one after the other.  It does @em not work
     *  with a pointer array, i.e. somthing that can be addresed by
     *  the data[x][y] notation!
     *
     *  @param sizeX Number of columns of the map data array.
     *
     *  @param sizeY Number of rows of the map data array.
     *
     *  @param rowMajor Flag if the data is organized by rows (all
     *  columns of the first row after eachother,then all columns of
     *  the second row etc.) or in column major mode.  If true, the
     *  rowMajor mode is assumed.
     */
    static void dumpDoubleToGP(FILE *fh, const char *title, double *data, int sizeX, int sizeY, bool rowMajor = true);
 
  private:
    // disable constructors and operators
    MapStore();
    MapStore(const MapStore &old);
    MapStore &operator=(const MapStore &old);

    /* The following private methods are documented in the
       implementation file. */
    bool convX(int &x) const;
    bool convY(int &y) const;
    void convXRange(int &x, int &dx) const;
    void convYRange(int &y, int &dy) const;

    int makeIdx(const int x, const int y) const;

    void internalFillRect(int x, int y, int dx, int dy, double val);

    int sizeX;
    int sizeY;
    int originX;
    int originY;
    double resolution;

    int maxIdx;

    double *data;
  };

}  // closes namespace mapstore

#endif // MAP_STORE_H
