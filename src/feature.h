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

#ifndef _FEATURE_H
#define _FEATURE_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <geos_c.h>

#include "palgeometry.h"

#include "pointset.h"
#include "feature_id.h"
#include "util.h"


namespace pal
{
  /** optional additional info about label (for curved labels) */
  class  LabelInfo
  {
    public:
      struct CharacterInfo {
        double width;
      };

      LabelInfo( int num, double height, double maxinangle = 20.0, double maxoutangle = -20.0 )
          : max_char_angle_inside(maxinangle)
          , max_char_angle_outside(maxoutangle)
          , label_height(height)
          , char_info(num)
      { }

      int char_num() const { return char_info.size(); }

      double max_char_angle_inside;
      double max_char_angle_outside;
      double label_height;
      std::vector<CharacterInfo> char_info;
  };

  class LabelPosition;
  class FeaturePart;

  class  Feature
  {
      friend class FeaturePart;
      friend class Layer;

    public:
      Feature(FeatureId id, PalGeometry* userG, double lx, double ly);
      ~Feature();
      /**
       * \brief create a feature
       *
       * @param geom_id unique identifier
       * @param label_x label width
       * @param label_y label height
       * @param userGeom user's geometry that implements the PalGeometry interface
       * @param labelPosX x position of the label (in case of fixed label position)
       * @param labelPosY y position of the label (in case of fixed label position)
       * @param fixedPos true if a single fixed position for this label is needed
       * @param angle fixed angle (in radians) to rotate the label
       * @param fixedAngle whether to use provided fixed angle
       * @param xQuadOffset move label to quadrant: left, don't move, right (-1, 0, 1)
       * @param yQuadOffset move label to quadrant: down, don't move, up (-1, 0, 1)
       * @param xOffset map unit (+/-) to x-offset the label
       * @param yOffset map unit (+/-) to y-offset the label
       * @param alwaysShow whether to skip priority and always show the label (causes overlapping)
       *
       * @throws PalException::FeatureExists
       *
       * @return nullptr if the parameters are invalid
       */
      static std::unique_ptr<Feature> create(FeatureId geom_id, PalGeometry *userGeom, double label_x, double label_y,
                               double labelPosX, double labelPosY, bool fixedPos, double angle, bool fixedAngle,
                               int xQuadOffset, int yQuadOffset, double xOffset, double yOffset, bool alwaysShow );

      /// @note takes ownership
      void setLabelInfo( LabelInfo* info ) { labelInfo.reset(info); }
      void setDistLabel( double dist ) { distlabel = dist; }
      //Set label position of the feature to fixed x/y values
      void setFixedPosition( double x, double y ) { fixedPos = true; fixedPosX = x; fixedPosY = y;}
      void setQuadOffset( double x, double y ) { quadOffset = true; quadOffsetX = x; quadOffsetY = y;}
      void setPosOffset( double x, double y ) { offsetPos = true; offsetPosX = x; offsetPosY = y;}
      bool fixedPosition() const { return fixedPos; }
      //Set label rotation to fixed value
      void setFixedAngle( double a ) { fixedRotation = true; fixedAngle = a; }
      void setAlwaysShow( bool bl ) { alwaysShow = bl; }

    protected:
      Layer *layer = nullptr;
      PalGeometry *userGeom;
      double label_x;
      double label_y;
      double distlabel = 0;
      std::unique_ptr<LabelInfo> labelInfo; // optional
      FeatureId uid;

      bool fixedPos = false; //true in case of fixed position (only 1 candidate position with cost 0)
      double fixedPosX = 0;
      double fixedPosY = 0;
      bool quadOffset = false; // true if a quadrant offset exists
      double quadOffsetX = 0;
      double quadOffsetY = 0;
      bool offsetPos = false; //true if position is to be offset by set amount
      double offsetPosX = 0;
      double offsetPosY = 0;
      //Fixed (e.g. data defined) angle only makes sense together with fixed position
      bool fixedRotation = false;
      double fixedAngle = false; //fixed angle value (in rad)
      bool alwaysShow = false; //true is label is to always be shown (but causes overlapping)
  };

  /**
   * \brief Main class to handle feature
   */
  class  FeaturePart : public PointSet
  {

    protected:
      Feature* f;

      int nbHoles;
      PointSet **holes;

      GEOSGeometry *the_geom;
      bool ownsGeom;

      /** \brief read coordinates from a GEOS geom */
      void extractCoords( const GEOSGeometry* geom );

      /** find duplicate (or nearly duplicate points) and remove them.
       * Probably to avoid numerical errors in geometry algorithms.
       */
      void removeDuplicatePoints();

    public:

      /**
        * \brief create a new generic feature
        *
        * \param feat a pointer for a Feat which contains the spatial entites
        */
      FeaturePart( Feature *feat, const GEOSGeometry* geom );

      /**
       * \brief Delete the feature
       */
      virtual ~FeaturePart();

      /**
       * \brief generate candidates for point feature
       * Generate candidates for point features
       * \param x x coordinates of the point
       * \param y y coordinates of the point
       * \param scale map scale is 1:scale
       * \param lPos pointer to an array of candidates, will be filled by generated candidates
       * \param angle orientation of the label
       * \return the number of generated cadidates
       */
      int setPositionForPoint( double x, double y, double scale, LabelPosition ***lPos, double delta_width, double angle );

      /**
       * generate one candidate over specified point
       */
      int setPositionOverPoint( double x, double y, double scale, LabelPosition ***lPos, double delta_width, double angle );

      /**
       * \brief generate candidates for line feature
       * Generate candidates for line features
       * \param scale map scale is 1:scale
       * \param lPos pointer to an array of candidates, will be filled by generated candidates
       * \param mapShape a pointer to the line
       * \return the number of generated cadidates
       */
      int setPositionForLine( double scale, LabelPosition ***lPos, PointSet *mapShape, double delta_width );

      LabelPosition* curvedPlacementAtOffset( PointSet* path_positions, double* path_distances,
                                              int orientation, int index, double distance );

      /**
       * Generate curved candidates for line features
       */
      int setPositionForLineCurved( LabelPosition ***lPos, PointSet* mapShape );

      /**
       * \brief generate candidates for point feature
       * Generate candidates for point features
       * \param scale map scale is 1:scale
       * \param lPos pointer to an array of candidates, will be filled by generated candidates
       * \param mapShape a pointer to the polygon
       * \return the number of generated cadidates
       */
      int setPositionForPolygon( double scale, LabelPosition ***lPos, PointSet *mapShape, double delta_width );


      /**
       * \brief Feature against problem bbox
       * \param bbox[0] problem x min
       * \param bbox[1] problem x max
       * \param bbox[2] problem y min
       * \param bbox[3] problem y max
       * return A set of feature which are in the bbox or null if the feature is in the bbox
       */
      //LinkedList<Feature*> *splitFeature( double bbox[4]);


      /**
       * \brief return the layer that feature belongs to
       * \return the layer of the feature
       */
      Layer * getLayer();

      /**
       * \brief generic method to generate candidates
       * This method will call either setPositionFromPoint(), setPositionFromLine or setPositionFromPolygon
       * \param scale the map scale is 1:scale
       * \param lPos pointer to candidates array in which candidates will be put
       * \param bbox_min min values of the map extent
       * \param bbox_max max values of the map extent
       * \param mapShape generate candidates for this spatial entites
       * \param candidates index for candidates
       * \return the number of candidates in *lPos
       */
      int setPosition(double scale,
                      LabelPosition*** lPos,
                      double bbox_min[2],
                      double bbox_max[2],
                      PointSet* mapShape,
                      RTree<LabelPosition*, double, 2, double>* candidates);

      /**
       * \brief get the unique id of the feature
       * \return the feature unique identifier
       */
      pal::FeatureId getUID();


      /**
       * \brief Print feature informations
       * Print feature unique id, geometry type, points, and holes on screen
       */
      void print();


      PalGeometry* getUserGeometry() { return f->userGeom; }

      void setLabelSize( double lx, double ly ) { f->label_x = lx; f->label_y = ly; }
      double getLabelWidth() const { return f->label_x; }
      double getLabelHeight() const { return f->label_y; }
      void setLabelDistance( double dist ) { f->distlabel = dist; }
      double getLabelDistance() const { return f->distlabel; }

      bool getFixedRotation() { return f->fixedRotation; }
      double getLabelAngle() { return f->fixedAngle; }
      bool getFixedPosition() { return f->fixedPos; }
      bool getAlwaysShow() { return f->alwaysShow; }

      int getNumSelfObstacles() const { return nbHoles; }
      PointSet* getSelfObstacle( int i ) { return holes[i]; }

      /** check whether this part is connected with some other part */
      bool isConnected( FeaturePart* p2 );

      /** merge other (connected) part with this one and save the result in this part (other is unchanged).
       * Return true on success, false if the feature wasn't modified */
      bool mergeWithFeaturePart( FeaturePart* other );

      void addSizePenalty( int nbp, LabelPosition** lPos, double bbx[4], double bby[4] );

  };

} // end namespace pal

#endif
