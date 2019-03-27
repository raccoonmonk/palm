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

#ifndef _LAYER_H_
#define _LAYER_H_

#include <fstream>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <memory>

#include "pal.h"
#include "palgeometry.h"
#include "feature_id.h"

namespace pal
{

  template <class Type> class LinkedList;
  template <class Type> class Cell;

  template<class DATATYPE, class ELEMTYPE, int NUMDIMS, class ELEMTYPEREAL, int TMAXNODES, int TMINNODES> class RTree;

  class Feature;
  class FeaturePart;
  class Pal;
  class LabelInfo;

  /**
   * \brief A layer of spacial entites
   *
   * a layer is a bog of feature with some data which influence the labelling process
   *
   *  \author Maxence Laurent <maxence _dot_ laurent _at_ heig-vd _dot_ ch>
   */
class  Layer {
      friend class Pal;
      friend class FeaturePart;

      friend class Problem;

      friend class LabelPosition;
      friend bool extractFeatCallback( FeaturePart *ft_ptr, void *ctx );
      friend void toSVGPath( int nbPoints, double *x, double *y, int dpi, Layer *layer, int type, char *uid, std::ostream &out, double scale, int xmin, int ymax, bool exportInfo, char *color );

    public:
      enum LabelMode { LabelPerFeature, LabelPerFeaturePart };
      enum UpsideDownLabels
      {
        Upright, // upside-down labels (90 <= angle < 270) are shown upright
        ShowDefined, // show upside down when rotation is layer- or data-defined
        ShowAll // show upside down for all labels, including dynamic ones
      };

      bool getDisplayAll() const { return displayAll; }

      /**
       * \brief Create a new layer
       *
       * @param lyrName layer's name
       * @param min_scale bellow this scale: no labeling
       * @param max_scale above this scale: no labeling
       * @param arrangement Arrangement mode : how to place candidates
       * @param label_unit Unit for labels sizes
       * @param defaultPriority layer's prioriry (0 is the best, 1 the worst)
       * @param obstacle 'true' will discourage other label to be placed above features of this layer
       * @param active is the layer is active (currently displayed)
       * @param toLabel the layer will be labeled whether toLablel is true
       * @param pal pointer to the pal object
       * @param displayAll if true, all features will be labelled even though overlaps occur
       *
       */
      Layer(std::string lyrName, double min_scale, double max_scale, Arrangement arrangement, double defaultPriority, bool obstacle, bool active, bool toLabel, Pal *pal, bool displayAll = false );

      /**
       * \brief Delete the layer
       */
      virtual ~Layer();

      /**
       * \brief check if the scal is in the scale range min_scale -> max_scale
       * @param scale the scale to check
       */
      bool isScaleValid( double scale );

      /** add newly creted feature part into r tree and to the list */
      void addFeaturePart( FeaturePart* fpart, const std::string & labelText);

    public:
      /**
       * \brief get the number of features into layer
       */
      int getNbFeatures();

      /**
       * \brief get layer's name
       */
      const std::string & getName() const;


      /**
       *  \brief get arrangement policy
       */
      Arrangement getArrangement();

      /**
       * \brief set arrangement policy
       *
       * @param arrangement arrangement policy
       */
      void setArrangement( Arrangement arrangement );

      unsigned long getArrangementFlags() const { return arrangementFlags; }
      void setArrangementFlags( unsigned long flags ) { arrangementFlags = flags; }

      /**
       * \brief activate or desactivate the layer
       *
       * active means "is currently display". When active is false
       * feature of this layer will never be used (neither for
       * labelling nor as obstacles)
       *
       * @param active turn the layer active (true) or inactive (false)
       */
      void setActive( bool active );

      /**
       * \brief return the layer's activity status
       */
      bool isActive();


      /**
       * \brief tell pal whether the layer has to be labelled.
       *
       * The layer will be labelled if and only if toLabel and isActive were set to true
       *
       * @param toLabel set to false disable lbelling this layer
       */
      void setToLabel( bool toLabel );


      /**
       * \brief return if the layer will be labelled or not
       */
      bool isToLabel();


      /**
       * \brief mark layer's features as obstacles
       *
       * Avoid putting labels over obstalces.
       * isActive must also be true to consider feature as obstacles,
       * otherwise they will be ignored
       */
      void setObstacle( bool obstacle );

      /**
       * \brief return the obstacle status
       */
      bool isObstacle();

      /**
       * \brief set the minimum valid scale, below this scale the layer will not be labelled
       *
       * Use -1 to disable
       */
      void setMinScale( double min_scale );

      /**
       * \brief return the minimum valid scale
       */
      double getMinScale();


      /**
       * \brief set the maximum valid scale, upon this scale the layer will not be labelled
       *
       * use -1 to disable
       */
      void setMaxScale( double max_scale );


      /**
       * \brief return the maximum valid scale
       */
      double getMaxScale();


      /**
       * \ brief set the layer priority
       *
       * The best priority is 0, the worst is 1
       * Should be links with a slider in a nice gui
       */
      void setPriority( double priority );


      /**
       * return the layer's priority
       */
      double getPriority();

      void setLabelMode( LabelMode m ) { mode = m; }
      LabelMode getLabelMode() const { return mode; }

      void setMergeConnectedLines( bool m ) { mergeLines = m; }
      bool getMergeConnectedLines() const { return mergeLines; }

      void setUpsidedownLabels( UpsideDownLabels ud ) { upsidedownLabels = ud; }
      UpsideDownLabels getUpsidedownLabels() const { return upsidedownLabels; }

      /**
       * \brief register a feature in the layer
       *
       * @throws PalException::FeatureExists
       *
       * @return true on success (i.e. valid geometry)
       *
       * @note takes ownership of the feature
       */
      bool registerFeature(std::unique_ptr<Feature> feature, const std::string &labelText);

      /** return pointer to feature or NULL if doesn't exist */
      Feature* getFeature(FeatureId id );

      /** join connected features with the same label text */
      void joinConnectedFeatures();

    protected:
      std::string name; /* unique */

      /** list of feature parts */
      LinkedList<FeaturePart*> *featureParts;

      /** list of features - for deletion */
      std::vector<std::unique_ptr<Feature>> features;

      Pal *pal;

      double defaultPriority;

      bool obstacle;
      bool active;
      bool toLabel;
      bool displayAll;

      double min_scale;
      double max_scale;

      /** optional flags used for some placement methods */
      Arrangement arrangement;
      unsigned long arrangementFlags;
      LabelMode mode;
      bool mergeLines;

      UpsideDownLabels upsidedownLabels = UpsideDownLabels::Upright;

      // indexes (spatial and id)
      RTree<FeaturePart*, double, 2, double, 8, 4> *rtree;
      std::unordered_map<FeatureId, Feature*> hashtable;

      std::unordered_map<std::string, LinkedList<FeaturePart*>*> connectedHashtable;
    std::vector<std::string> connectedTexts;
    std::mutex modMutex;
};

} // end namespace pal

#endif
