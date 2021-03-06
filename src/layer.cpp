/*
 *   libpal - Automated Placement of Labels Library
 *
 *   Copyright (C) 2008 Maxence Laurent, MIS-TIC, HEIG-VD
 *                      University of Applied Sciences, Western Switzerland
 *                      http://www.hes-so.ch,
 *                 2013 Semenov Andrey, Alexey Korzun, Artem Grishin
 *                      Esti-Map company
 *                      http://www.esti-map.ru/
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

#define _CRT_SECURE_NO_DEPRECATE

#include <stddef.h>
#include <geos_c.h>

#include <iostream>
#include <cstring>
#include <cmath>

#include "pal.h"
#include "layer.h"
#include "palexception.h"
#include "internalexception.h"

#include "linkedlist.hpp"

#include "feature.h"
#include "geomfunction.h"
#include "util.h"

namespace pal
{

  Layer::Layer(std::string lyrName, double min_scale, double max_scale, Arrangement arrangement, double defaultPriority, bool obstacle, bool active, bool toLabel, Pal *pal, bool displayAll )
      : name(std::move(lyrName)), pal( pal ), obstacle( obstacle ), active( active ),
      toLabel( toLabel ), displayAll( displayAll ),
      min_scale( min_scale ), max_scale( max_scale ),
      arrangement( arrangement ), arrangementFlags( 0 ), mode( LabelPerFeature ), mergeLines( false )
  {
    rtree = new RTree<FeaturePart*, double, 2, double>();

    if ( defaultPriority < 0.0001 )
      this->defaultPriority = 0.0001;
    else if ( defaultPriority > 1.0 )
      this->defaultPriority = 1.0;
    else
      this->defaultPriority = defaultPriority;
  }

  Layer::~Layer()
  {
    std::lock_guard<std::mutex> guard(modMutex);

    for (auto * part : featureParts) {
      delete part;
    }
    featureParts.clear();

    // features in the hashtable
    features.clear();

    delete rtree;

    hashtable.clear();
  }

  Feature* Layer::getFeature(FeatureId id)
  {
    auto findIter = hashtable.find(id);
    return (findIter != hashtable.end()) ? findIter->second : nullptr;
  }


  bool Layer::isScaleValid( double scale )
  {
    return ( scale >= min_scale || min_scale == -1 )
           && ( scale <= max_scale || max_scale == -1 );
  }


  int Layer::getNbFeatures()
  {
    return features.size();
  }

  const std::string &Layer::getName() const {
    return name;
  }

  Arrangement Layer::getArrangement()
  {
    return arrangement;
  }

  void Layer::setArrangement( Arrangement arrangement )
  {
    this->arrangement = arrangement;
  }


  bool Layer::isObstacle()
  {
    return obstacle;
  }

  bool Layer::isToLabel()
  {
    return toLabel;
  }

  bool Layer::isActive()
  {
    return active;
  }


  double Layer::getMinScale()
  {
    return min_scale;
  }

  double Layer::getMaxScale()
  {
    return max_scale;
  }

  double Layer::getPriority()
  {
    return defaultPriority;
  }

  void Layer::setObstacle( bool obstacle )
  {
    this->obstacle = obstacle;
  }

  void Layer::setActive( bool active )
  {
    this->active = active;
  }

  void Layer::setToLabel( bool toLabel )
  {
    this->toLabel = toLabel;
  }

  void Layer::setMinScale( double min_scale )
  {
    this->min_scale = min_scale;
  }

  void Layer::setMaxScale( double max_scale )
  {
    this->max_scale = max_scale;
  }

  void Layer::setPriority( double priority )
  {
    if ( priority >= 1.0 ) // low priority
      defaultPriority = 1.0;
    else if ( priority <= 0.0001 )
      defaultPriority = 0.0001; // high priority
    else
      defaultPriority = priority;
  }

  bool Layer::registerFeature(std::unique_ptr<Feature> feature, const std::string & labelText) {
    if (!feature)
      return false;

    const auto & geom_id = feature->uid;
    std::lock_guard<std::mutex> guard(modMutex);

    if (hashtable.find(geom_id) != hashtable.end()) {
      //A feature with this id already exists. Don't throw an exception as sometimes,
      //the same feature is added twice (dateline split with otf-reprojection)
      return false;
    }

    feature->layer = this;

    bool first_feat = true;

    double geom_size = -1, biggest_size = -1;
    FeaturePart* biggest_part = nullptr;

    // break the (possibly multi-part) geometry into simple geometries
    const GEOSGeometry *the_geom = feature->userGeom->getGeosGeometry();
    auto simpleGeometries = unmulti(the_geom);
    if (simpleGeometries.empty()) { // unmulti() failed?
      throw InternalException::UnknownGeometry();
    }

    featureParts.reserve(simpleGeometries.size());
    for (auto * geom : simpleGeometries) {
      // ignore invalid geometries (e.g. polygons with self-intersecting rings)
      if ( GEOSisValid( geom ) != 1 ) // 0=invalid, 1=valid, 2=exception
      {
        std::cerr << "ignoring invalid feature " << geom_id << std::endl;
        continue;
      }

      int type = GEOSGeomTypeId( geom );

      if ( type != GEOS_POINT && type != GEOS_LINESTRING && type != GEOS_POLYGON )
      {
        throw InternalException::UnknownGeometry();
      }

      FeaturePart* fpart = new FeaturePart(feature.get(), geom);

      // ignore invalid geometries
      if (( type == GEOS_LINESTRING && fpart->nbPoints < 2 ) ||
          ( type == GEOS_POLYGON && fpart->nbPoints < 3 ) )
      {
        delete fpart;
        continue;
      }

      // polygons: reorder coordinates
      if ( type == GEOS_POLYGON && reorderPolygon( fpart->nbPoints, fpart->x, fpart->y ) != 0 )
      {
        delete fpart;
        continue;
      }

      if ( mode == LabelPerFeature && ( type == GEOS_POLYGON || type == GEOS_LINESTRING ) )
      {
        if ( type == GEOS_LINESTRING )
          GEOSLength( geom, &geom_size );
        else if ( type == GEOS_POLYGON )
          GEOSArea( geom, &geom_size );

        if ( geom_size > biggest_size )
        {
          biggest_size = geom_size;
          delete biggest_part; // safe with NULL part
          biggest_part = fpart;
        } else {
          delete fpart;
        }
        continue; // don't add the feature part now, do it later
        // TODO: we should probably add also other parts to act just as obstacles
      }

      // feature part is ready!
      addFeaturePart( fpart, labelText );

      first_feat = false;
    }

    feature->userGeom->releaseGeosGeometry( the_geom );

    // if using only biggest parts...
    if (( mode == LabelPerFeature || feature->fixedPosition() ) && biggest_part != NULL )
    {
      addFeaturePart( biggest_part, labelText );
      first_feat = false;
    } else {
      delete biggest_part;
    }

    // add feature to layer if we have added something
    if (!first_feat) {
      hashtable.insert({geom_id, feature.get()});
      features.push_back(std::move(feature));
    }

    return !first_feat; // true if we've added something

  }

  void Layer::addFeaturePart( FeaturePart* fpart, const std::string & labelText )
  {
    double bmin[2];
    double bmax[2];
    fpart->getBoundingBox( bmin, bmax );

    // add to list of layer's feature parts
    featureParts.push_back(fpart);

    // add to r-tree for fast spatial access
    rtree->Insert( bmin, bmax, fpart );

    // add to hashtable with equally named feature parts
    if (mergeLines && !labelText.empty()) {
      LinkedList< FeaturePart*>* lst = nullptr;
      auto findIter = connectedHashtable.find(labelText);
      if (findIter == std::end(connectedHashtable)) {
        // entry doesn't exist yet
        lst = new LinkedList<FeaturePart*>(ptrFeaturePartCompare);
        connectedHashtable.insert({labelText, lst});
        connectedTexts.push_back(labelText);
      } else {
        lst = findIter->second;
      }
      lst->push_back(fpart); // add to the list
    }
  }

  static FeaturePart* _findConnectedPart( FeaturePart* partCheck, LinkedList<FeaturePart*>* otherParts )
  {
    // iterate in the rest of the parts with the same label
    Cell<FeaturePart*>* p = otherParts->getFirst();
    while ( p )
    {
      if ( partCheck->isConnected( p->item ) )
      {
        // stop checking for other connected parts
        return p->item;
      }
      p = p->next;
    }

    return NULL; // no connected part found...
  }

  void Layer::joinConnectedFeatures()
  {
    // go through all label texts
    for (const auto & labelText : connectedTexts) {
      //std::cerr << "JOIN: " << labelText << std::endl;
      auto findIter = connectedHashtable.find(labelText);
      if (findIter == std::end(connectedHashtable))
        continue; // shouldn't happen
      LinkedList<FeaturePart*>* parts = findIter->second;

      // go one-by-one part, try to merge
      while ( parts->size() )
      {
        // part we'll be checking against other in this round
        FeaturePart* partCheck = parts->pop_front();

        FeaturePart* otherPart = _findConnectedPart( partCheck, parts );
        if ( otherPart )
        {
          //std::cerr << "- connected " << partCheck << " with " << otherPart << std::endl;

          // remove partCheck from r-tree
          double bmin[2], bmax[2];
          partCheck->getBoundingBox( bmin, bmax );
          rtree->Remove( bmin, bmax, partCheck );

          otherPart->getBoundingBox( bmin, bmax );

          // merge points from partCheck to p->item
          if ( otherPart->mergeWithFeaturePart( partCheck ) )
          {
            // reinsert p->item to r-tree (probably not needed)
            rtree->Remove( bmin, bmax, otherPart );
            otherPart->getBoundingBox( bmin, bmax );
            rtree->Insert( bmin, bmax, otherPart );
          }
        }
      }

      // we're done processing feature parts with this particular label text
      delete parts;
    }

    // we're done processing connected fetures
    connectedTexts.clear();
  }



} // end namespace

