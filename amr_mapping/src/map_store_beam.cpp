/* -*- mode: c++; -*- */

/** @file map_store_beam.cpp Implementation of the beam class.
 *
 *  The beam class allows to trace a beam (or line) through some grid.
 */

#include <math.h>
#include "map_store_beam.h"
#include "map_store_error.h"

#ifdef DEBUG_BEAM
#include <iostream>
#endif

namespace mapstore {


  MapStoreBeam::MapStoreBeam(double x, double y, double dir_a, double length) : initPosX(x), initPosY(y), dir(dir_a), maxLen(length), y2Sum(0), initCellX(lround(x)), initCellY(lround(y)), lastCellX(0), lastCellY(0),  xHalfStep(false), yHalfStep(false) {

    if (length < 20) {
      length = 20;
    }

    if (maxLen <= 0) {
      // internaly we use -1 as unlimited flag, because maxLen = 0 is
      // used when we have a single-point line.
      maxLen = -1;
      endX = 0;
      endY = 0;
    } else {
      endX = lround(maxLen*cos(dir));
      endY = lround(maxLen*sin(dir));
    }

    xDelta = lround(length*cos(dir));
    yDelta = lround(length*sin(dir));

    init2ndStage();
  }

  MapStoreBeam::MapStoreBeam(int x1, int y1, int x2, int y2) : initPosX(x1), initPosY(y1), y2Sum(0), initCellX(x1), initCellY(y1), lastCellX(0), lastCellY(0),  xHalfStep(false), yHalfStep(false) {
    xDelta = x2 - x1;
    yDelta = y2 - y1;
    dir = atan2(yDelta, xDelta);
    maxLen = sqrt(xDelta*xDelta + yDelta*yDelta);
    endX = x2-x1;
    endY = y2-y1;
    init2ndStage();
  }

  void MapStoreBeam::reInit(int x1, int y1, int x2, int y2) {
    initPosX = x1;
    initPosY = y1;
    y2Sum = 0;
    initCellX = x1;
    initCellY = y1;
    lastCellX = 0;
    lastCellY = 0;
    xHalfStep = false;
    yHalfStep = false;

    xDelta = x2 - x1;
    yDelta = y2 - y1;
    dir = atan2(yDelta, xDelta);
    maxLen = sqrt(xDelta*xDelta + yDelta*yDelta);
    endX = x2-x1;
    endY = y2-y1;
    init2ndStage();
  }

  /** @internal Internal second part of constructor. */
  void MapStoreBeam::init2ndStage(void) {

#ifdef DEBUG_BEAM
    std::cout << "New beam " << (void *)this << ": (" << initCellX << ", " << initCellY << ") --> (" << (initCellX+endX) << ", " << (initCellY+endY) << "), dir = " << (dir*180/M_PI) << " len = " << maxLen << std::endl;
#endif
    // determine octant
    if (yDelta >= 0) {
      if (xDelta >= 0) {
	if (xDelta >= yDelta) {
	  octant = 0;
	} else {
	  octant = 1;
	  swap(lastCellX, lastCellY);
	  swap(xDelta, yDelta);
	  swap(endX, endY);
	}
      } else {
	xDelta = -xDelta;
	endX = -endX;
	if (xDelta >= yDelta) {
	  octant = 3;
	} else {
	  octant = 2;
	  swap(lastCellX, lastCellY);
	  swap(xDelta, yDelta);
	  swap(endX, endY);
	}
      }
    } else {
      yDelta = -yDelta;
      endY = -endY;
      if (xDelta >= 0) {
	if (xDelta >= yDelta) {
	  octant = 7;
	} else {
	  octant = 6;
	  swap(lastCellX, lastCellY);
	  swap(xDelta, yDelta);
	  swap(endX, endY);
	}
      } else {
	xDelta = -xDelta;
	endX = -endX;
	if (xDelta >= yDelta) {
	  octant = 4;
	} else {
	  octant = 5;
	  swap(lastCellX, lastCellY);
	  swap(xDelta, yDelta);
	  swap(endX, endY);
	}
      }
    }

    x2Delta = xDelta + xDelta;
    unlimited = (maxLen == -1);
  }

  void MapStoreBeam::swap(int &a, int &b) const {
    int t(a);
    a = b;
    b = t;
  }

  void MapStoreBeam::makeMapCoord(int &x, int &y, const int xIn, const int yIn) const {
    switch (octant) {
    case 0:
      x = initCellX + xIn;
      y = initCellY + yIn;
      break;
    case 1:
      x = initCellX + yIn;
      y = initCellY + xIn;
      break;
    case 2:
      x = initCellX - yIn;
      y = initCellY + xIn;
      break;
    case 3:
      x = initCellX - xIn;
      y = initCellY + yIn;
      break;
    case 4:
      x = initCellX - xIn;
      y = initCellY - yIn;
      break;
    case 5:
      x = initCellX - yIn;
      y = initCellY - xIn;
      break;
    case 6:
      x = initCellX + yIn;
      y = initCellY - xIn;
      break;
    case 7:
      x = initCellX + xIn;
      y = initCellY - yIn;
      break;
    default:
      throw MapStoreError(MapStoreError::Internal);
    }
  }

  void MapStoreBeam::makeMapCoord(int &x, int &y) const {
    makeMapCoord(x, y, lastCellX, lastCellY);
  }

  /* implemented in header file:
   * void MapStoreBeam::getInitCell(int &x, int &y) const; */


  /* implemented in heaxderf fiel:
   *  bool MapStoreBeam::getLastCell(int &x, int &y) const; */

  /* implemented in header file:
   * void MapStoreBeam::getEndCell(int &x, int &y) const; */

  /* implemented in header file:
   * double MapStoreBeam::getLen(void) const; */

  bool MapStoreBeam::nextCell(int &x, int &y) {
#ifdef DEBUG_BEAM
    bool res = nextCell2(x,y);
    if (res == false) {
      int tx,ty;
      makeMapCoord(tx, ty, endX, endY);
      std::cout << "beam " << (void *)this << " exceeded: end (" << tx << ", " << ty << ")  ret (" << x << ", " << y << ")" << std::endl;
    }
    return res;
  }

  /** @internal Internal debug function. */
  bool MapStoreBeam::nextCell2(int &x, int &y) {
#endif
    /*
     *  the basic equation is y = x*slope with slope = ydelta/xdelta.
     *  The problem is, we are going to use integer arithmetrics.  To
     *  simplify things we assume that xdelta > ydelta, which can
     *  always be guaranteed by an appropriate coordinate
     *  transformation (basicly one or two mirroring operations).  So
     *  the division and subsequent multiplication needs to be
     *  replaced by some series of addition and substraction, basicly
     *  implementing a "devision with remainder".  Because we assume
     *  xdelta > ydelta we know that we will advance x by one in each
     *  step, and we will advance y by zero or one.  To find the y
     *  steps, we reformulate the line equation, stating y =
     *  (x*ydelta)/xdelta.  We need to advance y, whenever then
     *  integer division (x*ydelta)/xdelta changes its result.  This
     *  is the case, whenever (x*ydelta) has grown by an amount of
     *  xdelta.
     */

    if (xDelta == 0) {
      makeMapCoord(x,y);
      return false;
    }

    if (yHalfStep) {
      // continue second half-step which had to move to right *and*
      // top.  Here we do the move to top.
      lastCellY += 1;
      yHalfStep = false;
      // following call set x and y according to lastCellX,Y and current octant
      makeMapCoord(x,y);
      return (unlimited || (lastCellX <= endX && lastCellY <= endY));
    }

    if (xHalfStep) {
      // perform second half-step after first one moved current
      // position one cell up.  Here we do the following move to the
      // right and update state y2Sum.
      y2Sum += yDelta;
      // update y2Sum according to basic line algorithm.
      y2Sum -= x2Delta;
      lastCellX += 1;
      xHalfStep = false;
      // following call set x and y according to lastCellX,Y and current octant
      makeMapCoord(x,y);
      return (unlimited || (lastCellX <= endX && lastCellY <= endY));
    }

    // first half-step
    y2Sum += yDelta;
    if (y2Sum > xDelta) {
      // line crossed border to next higher y-row of cells in this
      // first half-step.  Therefore, next cell is on top of the
      // current one, not to its side.  In other words: increment y
      // but *not* *yet* x.  X will be incremented in second
      // half-step.
      xHalfStep = true;
      lastCellY += 1;
      // following call set x and y according to lastCellX,Y and current octant
      makeMapCoord(x,y);
      //      return true;
      return (unlimited || (lastCellX <= endX && lastCellY <= endY));
    } else {
      // line still in current y-row, therefore move to the right.
      lastCellX += 1;
      // second half-step will decide if we additionally move up.
    }

    // second half-step
    y2Sum += yDelta;
    if (y2Sum > xDelta) {
      // line crossed border to next higher y-row of cells in
      // 2. half-step.  Therefore, next cell is to the right of the
      // current one, second-next on top of that.  In other words:
      // increment now x (done above after first half-step), and in
      // next step unconditionaly increment y.  Here we set a flag to
      // perform second half-step at next call.
      yHalfStep = true;
      // update y2Sum according to basic line algorithm.
      y2Sum -= x2Delta;
    }
    // following call set x and y according to lastCellX,Y and current octant
    makeMapCoord(x,y);
    return (unlimited || (lastCellX <= endX && lastCellY <= endY));
  }

} 

// End
