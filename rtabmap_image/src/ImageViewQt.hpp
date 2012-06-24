/*
 * ImageViewQt.hpp
 *
 *  Created on: 2012-06-20
 *      Author: mathieu
 */

#ifndef IMAGEVIEWQT_HPP_
#define IMAGEVIEWQT_HPP_

#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtGui/QApplication>
#include <QtGui/QWidget>
#include <QtGui/QPainter>
#include <QtGui/QToolTip>
#include <QtGui/QMenu>

#include <utilite/UPlot.h>

class RGBPlot: public UPlot
{
public:
	RGBPlot(int x, int y, QWidget * parent = 0) :
		x_(x),
		y_(y)
	{
		r_ = this->addCurve("R", Qt::red);
		g_ = this->addCurve("G", Qt::green);
		b_ = this->addCurve("B", Qt::blue);
	}
	~RGBPlot() {}
	void setPixel(int r, int g, int b)
	{
		r_->addValue(r);
		g_->addValue(g);
		b_->addValue(b);
	}
	int x() const {return x_;}
	int y() const {return y_;}
private:
	int x_;
	int y_;
	UPlotCurve * r_;
	UPlotCurve * g_;
	UPlotCurve * b_;
};

class ImageViewQt : public QWidget
{
	Q_OBJECT;
public:
	ImageViewQt(QWidget * parent = 0) : QWidget(parent)
	{
		this->setMouseTracking(true);
	}
	~ImageViewQt() {}

public slots:
	void setImage(const QImage & image)
	{
		if(pixmap_.width() != image.width() || pixmap_.height() != image.height())
		{
			for(QMap<QPair<int,int>, RGBPlot*>::iterator iter = pixelMap_.begin(); iter!=pixelMap_.end();)
			{
				RGBPlot * plot = *iter;
				iter = pixelMap_.erase(iter);
				delete plot;
			}
			this->setMinimumSize(image.width(), image.height());
			this->setGeometry(this->geometry().x(), this->geometry().y(), image.width(), image.height());
		}
		else
		{
			for(QMap<QPair<int,int>, RGBPlot*>::iterator iter = pixelMap_.begin(); iter!=pixelMap_.end();++iter)
			{
				QRgb rgb = image.pixel((*iter)->x(), (*iter)->y());
				(*iter)->setPixel(qRed(rgb), qGreen(rgb), qBlue(rgb));
			}
		}
		pixmap_ = QPixmap::fromImage(image);
		this->update();
	}

private:
	void computeScaleOffsets(float & scale, float & offsetX, float & offsetY)
	{
		scale = 1.0f;
		offsetX = 0.0f;
		offsetY = 0.0f;

		if(!pixmap_.isNull())
		{
			float w = pixmap_.width();
			float h = pixmap_.height();
			float widthRatio = float(this->rect().width()) / w;
			float heightRatio = float(this->rect().height()) / h;

			if(widthRatio < heightRatio)
			{
				scale = widthRatio;
			}
			else
			{
				scale = heightRatio;
			}

			w *= scale;
			h *= scale;

			if(w < this->rect().width())
			{
				offsetX = (this->rect().width() - w)/2.0f;
			}
			if(h < this->rect().height())
			{
				offsetY = (this->rect().height() - h)/2.0f;
			}
		}
	}
private slots:
	void removePlot(QObject * obj)
	{
		if(obj)
		{
			RGBPlot * plot = (RGBPlot*)obj;
			pixelMap_.remove(QPair<int,int>(plot->x(), plot->y()));
		}
	}

protected:
	virtual void paintEvent(QPaintEvent *event)
	{
		if(!pixmap_.isNull())
		{
			//Scale
			float ratio, offsetX, offsetY;
			this->computeScaleOffsets(ratio, offsetX, offsetY);
			QPainter painter(this);
			painter.translate(offsetX, offsetY);
			painter.scale(ratio, ratio);
			painter.drawPixmap(QPoint(0,0), pixmap_);
		}
	}

	virtual void mouseMoveEvent(QMouseEvent * event)
	{
		if(!pixmap_.isNull())
		{
			QPoint pos = this->mapFromGlobal(event->globalPos());

			float ratio, offsetX, offsetY;
			computeScaleOffsets(ratio, offsetX, offsetY);
			pos.rx()-=offsetX;
			pos.ry()-=offsetY;
			pos.rx()/=ratio;
			pos.ry()/=ratio;

			if(pos.x()>=0 && pos.x()<pixmap_.width() &&
			   pos.y()>=0 && pos.y()<pixmap_.height())
			{
				QToolTip::showText(event->globalPos(),
						QString("[%1,%2]")
						.arg(pos.x())
						.arg(pos.y()));
			}
		}
	}

	virtual void contextMenuEvent(QContextMenuEvent * event)
	{
		if(!pixmap_.isNull())
		{
			QPoint pos = this->mapFromGlobal(event->globalPos());

			float ratio, offsetX, offsetY;
			computeScaleOffsets(ratio, offsetX, offsetY);
			pos.rx()-=offsetX;
			pos.ry()-=offsetY;
			pos.rx()/=ratio;
			pos.ry()/=ratio;

			if(pos.x()>=0 && pos.x()<pixmap_.width() &&
			   pos.y()>=0 && pos.y()<pixmap_.height())
			{
				QMenu menu;
				QAction * a = menu.addAction(tr("Plot pixel (%1,%2) variation").arg(pos.x()).arg(pos.y()));
				QAction * b = menu.exec(event->globalPos());
				if(b == a)
				{
					if(!pixelMap_.contains(QPair<int,int>(pos.x(), pos.y())))
					{
						RGBPlot * plot = new RGBPlot(pos.x(), pos.y(), this);
						plot->setWindowTitle(tr("Pixel (%1,%2)").arg(pos.x()).arg(pos.y()));
						connect(plot, SIGNAL(destroyed(QObject *)), this, SLOT(removePlot(QObject *)));
						plot->setMaxVisibleItems(100);
						plot->show();
						pixelMap_.insert(QPair<int,int>(pos.x(), pos.y()), plot);
					}
				}

			}
		}
	}

private:
	QPixmap pixmap_;
	QMap<QPair<int,int>, RGBPlot*> pixelMap_;
};


#endif /* IMAGEVIEWQT_HPP_ */
