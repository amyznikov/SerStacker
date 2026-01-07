/*
 * QDataAnnotationSelectorCtrl.cc
 *
 *  Created on: Jul 3, 2025
 *      Author: gandriim
 */

#include "QDataAnnotationSelectorCtrl.h"
#include <core/debug.h>

QDataAnnotationSelectorCtrl::QDataAnnotationSelectorCtrl(QWidget * parent) :
  Base(parent)
{
  _form = new QFormLayout(this);

  c_data_annotation_labels & data_annotation =
      c_data_annotation_labels::default_instance();

  const int change_handler_id =
      data_annotation.add_change_handler([this]() {
        updateColormaps();
      });


  connect(this, &QObject::destroyed,
      [change_handler_id]() {
        c_data_annotation_labels::default_instance().remove_change_handler(change_handler_id);
      });

  updateColormaps();
}

void QDataAnnotationSelectorCtrl::updateColormaps()
{
  if ( !_selectors.empty() ) {
    _selectors.clear();
    while (_form->rowCount() > 0) {
      _form->removeRow(_form->rowCount() - 1);
    }
  }

  const std::vector<c_data_annotation_labels::ColorMap::uptr>& colormaps =
      c_data_annotation_labels::default_instance().colormaps();

  for ( size_t i = 0, n = colormaps.size(); i < n; ++i  ) {

    QComboBox * ctrl = new QComboBox(this);
    ctrl->setEditable(false);

    ctrl->addItem("DONT CHANGE", QVariant::fromValue((int)(-1)));

    const auto & colormap = colormaps[i];
    for ( auto ii = colormap->begin(); ii != colormap->end(); ++ii ) {
      const c_data_annotation_labels::Label & label = ii->second;
      ctrl->addItem(label.name.c_str(), QVariant::fromValue((int)(ii->first)));
    }

    _selectors.append(ctrl);
    _form->addRow(colormap->name().c_str(), ctrl);

    connect(ctrl, (void (QComboBox::*)(int)) (&QComboBox::currentIndexChanged),
        [this, ctrl, i](int cursel) {
          Q_EMIT annotationLabelChanged(i, ctrl->currentData().toInt());
        });
  }

}

void QDataAnnotationSelectorCtrl::setAnnotationLabel(int cmap, int label)
{
  if ( cmap >= 0 && cmap < _selectors.size() ) {
    QComboBox * ctrl = _selectors[cmap];

    QSignalBlocker block(ctrl);
    ctrl->setCurrentIndex(ctrl->findData(QVariant::fromValue((int)(label))));
  }
}

bool QDataAnnotationSelectorCtrl::getAnnotationLabel(int cmap, int *label) const
{
  if ( cmap >= 0 && cmap < _selectors.size() ) {
    *label = _selectors[cmap]->currentData().toInt();
    return true;
  }

  return false;
}
