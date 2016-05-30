/* -*- mode: c++; -*- */

/** @file map_store_error.h  Implementation of exception class for the sonar map system. */

#ifndef MAP_STORE_ERROR_H
#define MAP_STORE_ERROR_H

#include <stdexcept>


namespace mapstore {

  /** Describe an error condition in the sonar map classes. 
   *
   *  This class implements the error handling in the map classes
   *  using C++ exceptions.  Currently two error types are defined,
   *  Range and Internal.
   *
   *  To add your own error codes / conditions make a derived class
   *  with its own error type enumeration and variable and reimplement
   *  type2str().  Your constructor should then call the base
   *  constructor with a error type of "Extended" and an explanation
   *  string.
   */
  class MapStoreError : public std::exception {
  public:

    /** Defined error categories. */
    enum MapStoreErrorType {
      /** Some cell coordinates are outside the allowed range. */
      Range,

      /** Internal logic error.  Generally indicate a bug in the
       *  MapStore class and/or its support classes. */
      Internal,

      /** General unexplained error.  Used as a catch-all. */
      Generic,

      /** Reserved for derived classes.
       *
       *  This error code should be used in derived error classes to
       *  indicate that more specific information may be available
       *  in the derived class.
       */
      Extended,

	  /** File format error on loading a map from a file. */
	  Format
    };

    /** Construct error of given type @a err_a. */
    MapStoreError(MapStoreErrorType err_a) : err(err_a), err_str(type2str(err_a)) {
    }

    /** Construct error of given type @a err_a and explanation @a msg.
     *
     *  The message @a msg must be a static allocated string, it must
     *  @em not be a heap or stack object!
     *
     *  It is allowed to pass a Null pointer as @a msg.
     */
    MapStoreError(MapStoreErrorType err_a, const char *msg) : err(err_a), err_str(msg) {
      if (msg == 0)
	msg = type2str(Generic);
    }

    /** Destructor - does nothing as no resources are used. */
    virtual ~MapStoreError() throw() {}

    /** Return descriptive error message, if any. */
    virtual const char *what() const throw() {
      return err_str;
    }

    /** Return numerical exception type. */
    MapStoreErrorType getType(void) const throw() {
      return err;
    }

    /** Convert error type into generic description for that error. */
    static const char *type2str(MapStoreErrorType type) {
      switch (type) {
      case Generic:
	return "(unknown error)";
      case Internal:
	return "Internal error detected.  Probably a bug in MapStore and/or its suport classes.";
      case Extended:
	return "User-defined error found.";
      case Range:
	return "cell coordinates out of range";
      default:
	return "(?)";
      }
    }

  private:
    /** Disabled default constructor. */
    MapStoreError();

    /** Type of error reported. */
    MapStoreErrorType err;

    /** Textual explanation of error. */
    const char *err_str;
  };

}

#endif
