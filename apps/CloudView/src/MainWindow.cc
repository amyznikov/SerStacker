/*
 * MainWindow.cc
 *
 *  Created at Sat, November 18, 2023
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/QToolbarSpacer.h>
#include <gui/widgets/QMenuWidgetAction.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qimagesave/QImageSaveOptions.h>
#include <gui/qthumbnailsview/QThumbnails.h>
#include <gui/qimproc/QImageProcessorsCollection.h>
#include <gui/qpipeline/QImageProcessingPipeline.h>
#include <gui/qpipeline/QPipelineThread.h>
#include <core/io/load_image.h>
#include <core/debug.h>

#define ICON_reload             ":/gui/icons/reload"
#define ICON_prev               ":/gui/icons/prev"
#define ICON_next               ":/gui/icons/next"
#define ICON_like               ":/gui/icons/like"
#define ICON_dislike            ":/gui/icons/dislike"
#define ICON_close              ":/gui/icons/close"
#define ICON_histogram          ":/gui/icons/histogram"
#define ICON_marker_blue        ":/gui/icons/marker-blue"
#define ICON_reference          ":/gui/icons/reference"
#define ICON_options            ":/gui/icons/options"
#define ICON_mask               ":/gui/icons/mask"
#define ICON_frame              ":/gui/icons/frame"
#define ICON_badframe           ":/gui/icons/badframe"

#define ICON_copy               ":/gui/icons/copy"
#define ICON_delete             ":/gui/icons/delete"

#define ICON_roi                ":/serstacker/icons/roi.png"
#define ICON_point_size         ":/serstacker/icons/degree.png"
#define ICON_brightness         ":/serstacker/icons/brightness.png"
#define ICON_cloud_rotate       ":/serstacker/icons/cloud_rotate.png"
#define ICON_cloud_view_target  ":/serstacker/icons/cloud_view_target.png"


namespace cloudview {
///////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow()
{

  setWindowIcon(QIcon(":/cloudview/icons/app-icon.png"));
  updateWindowTittle();
  QImageProcessorsCollection::load();


  ///////////////////////////////////
  // Setup docking views

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner( Qt::TopLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::TopRightCorner, Qt::RightDockWidgetArea );
  setCorner( Qt::BottomLeftCorner, Qt::LeftDockWidgetArea );
  setCorner( Qt::BottomRightCorner, Qt::BottomDockWidgetArea );


  restoreState();
}

MainWindow::~MainWindow()
{
  saveState();
}

void MainWindow::updateWindowTittle()
{
  setWindowTitle("CloudView");
}



///////////////////////////////////////////////////////////////////////////////
}  // namespace cloudview
