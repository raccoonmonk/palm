/*
 *   libpal - Automated Placement of Labels Library
 *
 *   Copyright (C) 2008 Maxence Laurent, MIS-TIC, HEIG-VD
 *                      University of Applied Sciences, Western Switzerland
 *                      http://www.hes-so.ch
 *
 *   Contact:
 *      maxence.laurent <at> heig-vd <dot> ch
 *    or
 *      eric.taillard <at> heig-vd <dot> ch
 *
 * This file is part of libpal.
 *
 * libpal is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libpal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libpal.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _UTIL_H
#define _UTIL_H

#include <cmath>
#include <cstring>
#include <geos_c.h>

#include "pal.h"

#include "rtree.hpp"
#include "pointset.h"

namespace pal
{
  class LabelPosition;
  class Layer;
  class FeaturePart;

  std::vector<const GEOSGeometry *> unmulti( const GEOSGeometry* the_geom );

  /**
   * \brief For usage in problem solving algorithm
   */
  class Feats
  {
    public:
      FeaturePart *feature;
      PointSet *shape;
      double priority;
      int nblp;
      LabelPosition **lPos;
  };


  typedef struct _elementary_transformation
  {
    int feat;
    int  old_label;
    int  new_label;
  } ElemTrans;



#define EPSILON 1e-9

template<typename T>
bool is_close_to_zero(T value) {
  static_assert(std::is_floating_point<T>::value, "only for floating point");
  return std::abs(value) < std::numeric_limits<decltype(value)>::epsilon();
}

  inline double degree2meter( double delta_deg )
  {
    double lat = delta_deg * 0.5;
    const static double rads = ( 4.0 * atan( 1.0 ) ) / 180.0;
    double a = cos( lat * rads );
    a = a * a;
    double c = 2.0 * atan2( sqrt( a ), sqrt( 1.0 - a ) );
    const static double ra = 6378000; // [m]
    const static double e = 0.0810820288;
    double radius = ra * ( 1.0 - e * e ) / pow( 1.0 - e * e * sin( lat * rads ) * sin( lat * rads ), 1.5 );
    double meters = ( delta_deg ) / 180.0 * radius * c; // [m]

    return meters;
  }

  /* From meters to PostScript Point */
  inline void convert2pt( int *x, double scale, int dpi )
  {
    *x = ( int )((( double ) * x / scale ) * 39.3700787402 * dpi + 0.5 );
  }


  inline int convert2pt( double x, double scale, int dpi )
  {
    return ( int )(( x / scale ) * 39.3700787402 * dpi + 0.5 );
  }


  void sort( double* heap, int* x, int* y, int N );


  inline bool intCompare( int a, int b )
  {
    return a == b;
  }

  inline bool strCompare( char * a, char * b )
  {
    return strcmp( a, b ) == 0;
  }

  inline bool ptrLPosCompare( LabelPosition * a, LabelPosition * b )
  {
    return a == b;
  }

  inline bool ptrPSetCompare( PointSet * a, PointSet * b )
  {
    return a == b;
  }

  inline bool ptrFeatureCompare( Feature * a, Feature * b )
  {
    return a == b;
  }
  inline bool ptrFeaturePartCompare( FeaturePart * a, FeaturePart * b )
  {
    return a == b;
  }

  inline bool ptrFeatsCompare( Feats * a, Feats * b )
  {
    return a == b;
  }

  inline bool ptrLayerCompare( Layer * a, Layer * b )
  {
    return a == b;
  }


  inline bool ptrETCompare( ElemTrans * a, ElemTrans * b )
  {
    return a == b;
  }

  /**
   * \brief Sort an array of pointers
   * \param items arays of pointers to sort
   * \param N number of items
   * \param greater function to compare two items
   **/
  void sort( void** items, int N, bool ( *greater )( void *l, void *r ) );

} // namespace

#endif
