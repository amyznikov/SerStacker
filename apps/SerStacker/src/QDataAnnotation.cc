/*
 * QDataAnnotation.cc
 *
 *  Created on: Oct 23, 2024
 *      Author: gandriim
 */

#include "QDataAnnotation.h"

namespace serstacker {

using ColorMap =
    c_data_annotation_labels::ColorMap::uptr;

using Label =
    c_data_annotation_labels::Label;

///////////////////////////////////////////////////////////////////////////////////////////////////

QDataAnnotationSettingsWidget::QDataAnnotationSettingsWidget(QWidget *parent) :
    Base(parent)
{
  colorMapSelection_ctl =
      add_combobox<QComboBox>("Select Colormap:", "",
          false,
          [this](int cursel, QComboBox*/*combo*/) {
            onCurrentColormapMapChanged(cursel);
          });

  labelSelection_ctl =
      add_combobox<QComboBox>("Select Label:", "",
          false,
          [this](int cursel, QComboBox*) {
            onCurrentLabelSelectionChanged(cursel);
          });

  labelValue_ctl =
      add_spinbox("Label Value:",
          "");

  labelValue_ctl->setRange(0, 255);
  labelValue_ctl->setReadOnly(true);

  labelColor_ctl =
      add_color_picker_button("Label Color:",
          "",
          [this](const QColor &v) {

            if ( _opts ) {

              const int currentColormapIndex = colorMapSelection_ctl->currentIndex();
              if (currentColormapIndex>=0 && currentColormapIndex < _opts->num_colormaps() ) {
                const ColorMap & colormap = _opts->colormap(currentColormapIndex);
                cv::Vec4b & color = colormap->at((uint8_t)labelValue_ctl->value()).color;
                color[0] = v.blue();
                color[1] = v.green();
                color[2] = v.red();
                Q_EMIT parameterChanged();
              }
            }
          });


  labelBlendAlpha_ctl =
    add_double_spinbox("Color Blend Alpha:",
        "Brush depth in [m]",
        [this](double value) {
            if ( _opts ) {
              const int currentColormapIndex = colorMapSelection_ctl->currentIndex();
              if (currentColormapIndex>=0 && currentColormapIndex < _opts->num_colormaps() ) {
                const ColorMap & colormap = _opts->colormap(currentColormapIndex);
                cv::Vec4b & color = colormap->at((uint8_t)labelValue_ctl->value()).color;
                color[3] = (uint8_t)(255 * std::max(0., std::min(value, 1.)));
                Q_EMIT parameterChanged();
              }
            }
        });

  labelBlendAlpha_ctl->setRange(0, 1);
  labelBlendAlpha_ctl->setSingleStep(1e-2);
  labelBlendAlpha_ctl->setDecimals(2);
  labelBlendAlpha_ctl->setValue(0.8);

  updateControls();
}


void QDataAnnotationSettingsWidget::setOpts(OptsType * opts)
{
  this->_opts = opts;
  populateColormaps();
  updateControls();
}

void QDataAnnotationSettingsWidget::populateColormaps()
{
  QSignalBlocker block(colorMapSelection_ctl);

  colorMapSelection_ctl->clear();
  if ( _opts ) {
    for ( const auto & colormap : _opts->colormaps() ) {
      colorMapSelection_ctl->addItem(colormap->name().c_str());
    }
  }

  if (colorMapSelection_ctl->count() > 0) {
    colorMapSelection_ctl->setCurrentIndex(0);
    onCurrentColormapMapChanged(colorMapSelection_ctl->currentIndex());
  }

}


void QDataAnnotationSettingsWidget::onCurrentColormapMapChanged(int cursel)
{
  labelSelection_ctl->blockSignals(true);

  labelSelection_ctl->clear();

  if ( !_opts || cursel < 0 || cursel >= _opts->num_colormaps() ) {
    labelSelection_ctl->setEnabled(false);
    labelValue_ctl->setEnabled(false);
    labelColor_ctl->setEnabled(false);
  }
  else {

    const ColorMap & colormap =
        _opts->colormap(cursel);

    for (auto ii = colormap->begin(); ii != colormap->end(); ++ii) {

      const auto & label_name =
          ii->second.name;

      const uint8_t label_value =
          ii->first;

      labelSelection_ctl->addItem(label_name.c_str(),
          QVariant::fromValue(label_value));

    }

    if ( labelSelection_ctl->count()  < 1 ) {
      labelSelection_ctl->setEnabled(false);
      labelValue_ctl->setEnabled(false);
      labelColor_ctl->setEnabled(false);
    }
    else {
      labelSelection_ctl->setCurrentIndex(0);
      labelSelection_ctl->setEnabled(true);
      labelValue_ctl->setEnabled(true);
      labelColor_ctl->setEnabled(true);
    }
  }

  labelSelection_ctl->blockSignals(false);

  onCurrentLabelSelectionChanged(labelSelection_ctl->currentIndex());
}

void QDataAnnotationSettingsWidget::onCurrentLabelSelectionChanged(int currentLabelIndex)
{
  if (_opts) {

    const int currentColormapIndex =
        colorMapSelection_ctl->currentIndex();

    if (currentColormapIndex >= 0 && currentColormapIndex < _opts->num_colormaps()) {

      const ColorMap &colormap =
          _opts->colormap(currentColormapIndex);

      if (currentLabelIndex >= 0 && currentLabelIndex < colormap->size()) {

        const uint8_t label_value =
            labelSelection_ctl->currentData().value<uint8_t>();

        const Label &label =
            colormap->at(label_value);

        QSignalBlocker labelValueBlock(labelValue_ctl);
        QSignalBlocker labelColorBlock(labelColor_ctl);
        QSignalBlocker labelBlendAlphaBlock(labelBlendAlpha_ctl);

        labelValue_ctl->setValue(label_value);
        labelColor_ctl->setColor(QColor(label.color[2], label.color[1], label.color[0]));
        labelBlendAlpha_ctl->setValue(label.color[3] / 255.);
      }
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

} /* namespace serstacker */
