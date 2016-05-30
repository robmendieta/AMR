/* -*- mode: c++; -*- */

/** @file map_store_cone.cpp Implement an iterator over a circular sector. */

#include <assert.h>
#include "map_store_cone.h"

#ifdef DEBUG_CONE
#include <iostream>
using std::cout;
using std::endl;
#endif

namespace mapstore {

  MapStoreCone::MapStoreCone(double x, double y, double dir, double width, double len) : beamInfoCenter(x, y, dir, len), beamInfoLeft(x, y, dir+width/2, len), beamInfoRight(x, y, dir-width/2, len), beamInfoLeftFill(x,y,x,y), beamInfoRightFill(x,y,x,y) {
    
    beamInfoCenter.getInitCell(xIdxCenter, yIdxCenter);
    beamInfoLeft.getInitCell(xIdxLeft, yIdxLeft);
    beamInfoRight.getInitCell(xIdxRight, yIdxRight);

    state = 0;
    leftInvalid = false;
    rightInvalid = false;

    minX = xIdxCenter;
    minY = yIdxCenter;
    sizeX = minX;
    sizeY = minY;

    int tx, ty;
    beamInfoCenter.getEndCell(tx, ty);
    if (tx < minX)
      minX = tx;
    if (ty < minY)
      minY = ty;
    if (tx > sizeX)
      sizeX = tx;
    if (ty > sizeY)
      sizeY = ty;

    beamInfoLeft.getEndCell(tx, ty);
    if (tx < minX)
      minX = tx;
    if (ty < minY)
      minY = ty;
    if (tx > sizeX)
      sizeX = tx;
    if (ty > sizeY)
      sizeY = ty;

    beamInfoRight.getEndCell(tx, ty);
    if (tx < minX)
      minX = tx;
    if (ty < minY)
      minY = ty;
    if (tx > sizeX)
      sizeX = tx;
    if (ty > sizeY)
      sizeY = ty;

    minX -= 4;
    minY -= 4;
    sizeX += 4;
    sizeY += 4;

#ifdef DEBUG_CONE
    cout << "ConeIterator: field: (" << minX << ", " << minY << ") --> (" << sizeX << ", " << sizeY << ")" << endl;
#endif
    sizeX -= minX;
    sizeY -= minY;

    sizeXRow = sizeX >> 4;
    sizeXRow += 1;

    cache = new int[sizeXRow*sizeY];
    int i;
    for (i = 0; i < sizeXRow*sizeY; i++)
      cache[i] = 0;
    cachehit = 0;
    cachemiss = 0;
  }

  MapStoreCone::~MapStoreCone() {
#ifdef DEBUG_CONE
    std::cout << "Cache hit " << cachehit << " cells " << cachemiss << std::endl;
#endif
    delete[] cache;
  }

  bool MapStoreCone::nextCell(int &x, int &y) {

    while (state != 9) {

      if (state == 0) {
	// return initial cell
	x = xIdxCenter;
	y = yIdxCenter;
	markDone(x,y);
	state = 1;
	return true;
      }

      if (state == 1) {
	// advance center beam
	if (beamInfoCenter.nextCell(xIdxCenter, yIdxCenter)) {
	  beamInfoLeftFill.reInit(xIdxCenter, yIdxCenter, xIdxLeft, yIdxLeft);
	  beamInfoRightFill.reInit(xIdxCenter, yIdxCenter, xIdxRight, yIdxRight);

	  x = xIdxCenter;
	  y = yIdxCenter;

	  state = 2;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	  if (markDone(x,y)) {
	    return true;
	  }
	} else {
	  state = 9;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	}
      }

      if (state == 2 && leftInvalid)
	state = 3;

      if (state == 2) {
	// iterate from center to left side
	while (beamInfoLeftFill.nextCell(xIdxLeft, yIdxLeft)) {
	  if (markDone(xIdxLeft, yIdxLeft)) {
	    x = xIdxLeft;
	    y = yIdxLeft;
	    return true;
	  }
	}

	state = 3;
#ifdef DEBUG_CONE
	cout << "next state: "  << state << endl;
#endif
      }

      if (state == 3 && rightInvalid)
	state = 4;

      if (state == 3) {
	// iterate from center to right side
	while (beamInfoRightFill.nextCell(xIdxRight, yIdxRight)) {
	  if (markDone(xIdxRight, yIdxRight)) {
	    x = xIdxRight;
	    y = yIdxRight;
	    return true;
	  }
	}

	state = 4;
#ifdef DEBUG_CONE
	cout << "next state: "  << state << endl;
#endif
      }

      if (state == 4 && leftInvalid)
	state = 6;

      if (state == 4) {
	// advance left border beam

	if (beamInfoLeft.getLen() < beamInfoCenter.getLen()) {
	  if (beamInfoLeft.nextCell(xIdxLeft, yIdxLeft)) {
	    beamInfoLeftFill.reInit(xIdxLeft, yIdxLeft, xIdxCenter, yIdxCenter);
	    state = 5;
#ifdef DEBUG_CONE
	    cout << "next state: "  << state << endl;
#endif
	    if (markDone(xIdxLeft, yIdxLeft)) {
	      x = xIdxLeft;
	      y = yIdxLeft;
	      return true;
	    }
	  } else {
	    // xIdxLeft, yIdxLeft are invalid, left border reached its end
	    leftInvalid = true;
	    state = 6;
#ifdef DEBUG_CONE
	    cout << "next state: "  << state << endl;
#endif
	  }
	} else {
	  // left border could not be advanced -> skip iteration from
	  // left border to center, since it would be same as the
	  // already done iteration from center to left border.
	  state = 6;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	  // make sure (x,y)IdxLeft are set to current position on left
	  // beam, as expected by state 1
	  beamInfoLeft.getLastCell(xIdxLeft, yIdxLeft);
	}
      }

      if (state == 5 && leftInvalid)
	state = 6;

      if (state == 5) {
	// iterate from left side to center

	while (beamInfoLeftFill.nextCell(xIdxLeft, yIdxLeft)) {
	  if (markDone(xIdxLeft, yIdxLeft)) {
	    x = xIdxLeft;
	    y = yIdxLeft;
	    return true;
	  }
	}

	state = 6;
#ifdef DEBUG_CONE
	cout << "next state: "  << state << endl;
#endif
      }

      if (state == 6 && rightInvalid)
	state = 8;

      if (state == 6) {
	// advance right border beam

	if (beamInfoRight.getLen() < beamInfoCenter.getLen()) {
	  if (beamInfoRight.nextCell(xIdxRight, yIdxRight)) {
	    beamInfoRightFill.reInit(xIdxRight, yIdxRight, xIdxCenter, yIdxCenter);
	    state = 7;
#ifdef DEBUG_CONE
	    cout << "next state: "  << state << endl;
#endif
	    if (markDone(xIdxRight, yIdxRight)) {
	      x = xIdxRight;
	      y = yIdxRight;
	      return true;
	    }
	  } else {
	    // xIdxRight, yIdxRight are invalid, right border reached its end
	    rightInvalid = true;
	    state = 8;
#ifdef DEBUG_CONE
	    cout << "next state: "  << state << endl;
#endif
	  }
	} else {
	  // right border could not be advanced -> skip iteration from
	  // right border to center, since it would be same as the
	  // already done iteration from center to right border.
	  state = 8;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	  // make sure (x,y)IdxRight are set to current position on right
	  // beam, as expected by state 1
	  beamInfoRight.getLastCell(xIdxRight, yIdxRight);
	}
      }

      if (state == 7 && rightInvalid)
	state = 8;

      if (state == 7) {
	// iterate from right side to center

	while (beamInfoRightFill.nextCell(xIdxRight, yIdxRight)) {
	  if (markDone(xIdxRight, yIdxRight)) {
	    x = xIdxRight;
	    y = yIdxRight;
	    return true;
	  }
	}

	// go back to state 4, so left and right border beam can be
	// advanced until they exceed current length of center beam.
	state = 4;
#ifdef DEBUG_CONE
	cout << "next state: "  << state << endl;
#endif
      }

      if (state == 8) {
	// a right beam exceeding current length of center beam can
	// short-circute advancement of a left beam not yet at the
	// current length of the center beam.  Check if left beam is
	// already at its limit
	if (beamInfoLeft.getLen() < beamInfoCenter.getLen() && !leftInvalid) {
	  state = 4;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	} else {
	  state = 1;
#ifdef DEBUG_CONE
	  cout << "next state: "  << state << endl;
#endif
	}
      }
    }

    beamInfoCenter.getLastCell(x,y);
    return false;
  }

  bool MapStoreCone::markDone(int x, int y) {
#ifdef DEBUG_CONE
    cout << "markDone (" << x << ", " << y << ")" << endl;
#endif
    x -= minX;
    assert( (0 <= x) && (x < sizeX) );
    y -= minY;
    assert( (0 <= y) && (y < sizeY) );

    int xWord = x >> 4;
    int xBit = x & 0x0f;

    int idx = y*sizeXRow + xWord;
    bool res = ((cache[idx] & (1 << xBit)) == 0);
    if (res) {
      cache[idx] |= (1 << xBit);
      cachemiss += 1;
    } else {
      cachehit += 1;
    }
    return res;
  }

};

// End
