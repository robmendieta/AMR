#include "map_store_cone.h"
#include "map_store.h"
#include <boost/python.hpp>


namespace bpl = boost::python;

inline bpl::object pass_through(bpl::object const& o) { return o; }

/*
The class exposes MapStoreCone as python iterator. __iter__() and next() methods are
implemented.
The infos on bindings:
https://github.com/ethz-asl/programming_guidelines/wiki/Adding-python-bindings-to-your-cpp-catkin-package

iterator:
https://wiki.python.org/moin/boost.python/iterator
*/
class MapStoreConeWrapper
{
  public:

    MapStoreConeWrapper(double x, double y, double dir, double width, double len)
    :x_(x)
    ,y_(y)
    ,dir_(dir)
    ,width_(width)
    ,len_(len)
    ,map_store_cone_(NULL)
    {
      map_store_cone_ = new mapstore::MapStoreCone(x_, y_, dir_, width_, len_);
    }

    ~MapStoreConeWrapper()
    {
      delete map_store_cone_;
      map_store_cone_ = NULL;
    }

    bpl::api::object next()
    {
      int c_x;
      int c_y;
      if (map_store_cone_->nextCell(c_x, c_y))
      {
      	return bpl::make_tuple(c_x, c_y);
      } else
      {
        delete map_store_cone_;
        map_store_cone_ = new mapstore::MapStoreCone(x_, y_, dir_, width_, len_); // TODO: make reset in the __iter__ call !
        // renew map_store cone for further usage
        PyErr_SetString(PyExc_StopIteration, "No more data.");
        boost::python::throw_error_already_set();
        return bpl::api::object();
      }
    }

  private:
    double x_, y_, dir_, width_, len_;
    mapstore::MapStoreCone* map_store_cone_;
};


class MapStoreWrapper : public mapstore::MapStore
{
  public:

    MapStoreWrapper(int initSizeX, int initSizeY) : mapstore::MapStore(initSizeX, initSizeY) {}
    
    bpl::list getPublishData(double min, double max)
    {
      const double EPSILON = 1e-5;
      const double range = max - min;
      int width = this->getSizeX();
      int height = this->getSizeY();
      bpl::list pyList;
      const double* data = this->getRawData();

      /*
      for (int n = 0; n < numCells; n++) 
      {
        pyList.append(data[n]);
      }
      return pyList;
      */

      int i = 0;
      for (int x = 0; x < width; x++)
        for (int y = 0; y < height; y++, i++)
        {
          double d = data[x * height + y];
          if (d > max - EPSILON)
            //grid_msg->data[i] = 100;
            pyList.append(100);
          else if (d < min + EPSILON)
            //grid_msg->data[i] = 0;
            pyList.append(0);
          else
            //grid_msg->data[i] = (d - min) / range * 100;
            pyList.append((d - min) / range * 100);
        }
      return pyList;
    }



};



BOOST_PYTHON_MODULE(map_store_py)
{
    using namespace bpl;

    class_< MapStoreConeWrapper>("MapStoreCone", init<double, double, double, double, double>("MapStoreCone(x, y, dir, width, len)"))
    .def("next", &MapStoreConeWrapper::next, "next cell in the iterator")
    .def("__iter__", pass_through)
    ;

    class_< MapStoreWrapper,  boost::noncopyable>("MapStore", init<int, int>("MapStore(size_x, size_y"))
    .def("get", &MapStoreWrapper::get, "")
    .def("set", &MapStoreWrapper::set, "")
    .def("is_in_x_range", &MapStoreWrapper::isInX, "")
    .def("is_in_y_range", &MapStoreWrapper::isInY, "")
    .def("get_publish_data", &MapStoreWrapper::getPublishData, "get_publish_data(min_val, max_val)")
    ;

}