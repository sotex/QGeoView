/***************************************************************************
 * QGeoView is a Qt / C ++ widget for visualizing geographic data.
 * Copyright (C) 2021, Liu Yimin (ymwh@foxmail.com).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, see https://www.gnu.org/licenses.
 ****************************************************************************/

#include "QGVProjectionEPSG4490.h"

#include <QLineF>
#include <QtMath>

QGVProjectionEPSG4490::QGVProjectionEPSG4490()
        : QGVProjection("EPSG4490",
                        "China Geodetic Coordinate System 2000",
                        "Geodetic coordinate system for China - onshore and offshore. Adopted July 2008."
                        " Replaces Xian 1980 (CRS code 4610). Horizontal component of 3D system.")
{
    mEarthRadius = 6378137.0; /* meters */
    mGeoBoundary = QGV::GeoRect(90, -180, -90, +180);
    mProjBoundary = geoToProj(mGeoBoundary);
}

QGV::GeoRect QGVProjectionEPSG4490::boundaryGeoRect() const
{
    return mGeoBoundary;
}

QRectF QGVProjectionEPSG4490::boundaryProjRect() const
{
    return mProjBoundary;
}

QPointF QGVProjectionEPSG4490::geoToProj(const QGV::GeoPos& geoPos) const
{
    const double lon = geoPos.longitude();
    const double lat = geoPos.latitude();

    return QPointF(lon, lat);
}

QGV::GeoPos QGVProjectionEPSG4490::projToGeo(const QPointF& projPos) const
{
    const double lon = projPos.x();
    const double lat = projPos.y();
    return QGV::GeoPos(lat, lon);
}

QRectF QGVProjectionEPSG4490::geoToProj(const QGV::GeoRect& geoRect) const
{
    QRectF rect;
    rect.setTopLeft(geoToProj(geoRect.topLeft()));
    rect.setBottomRight(geoToProj(geoRect.bottomRight()));
    return rect;
}

QGV::GeoRect QGVProjectionEPSG4490::projToGeo(const QRectF& projRect) const
{
    return QGV::GeoRect(projToGeo(projRect.topLeft()), projToGeo(projRect.bottomRight()));
}

double QGVProjectionEPSG4490::geodesicMeters(const QPointF& projPos1, const QPointF& projPos2) const
{
    const QGV::GeoPos geoPos1 = projToGeo(projPos1);
    const QGV::GeoPos geoPos2 = projToGeo(projPos2);
    const double latitudeArc = (geoPos1.latitude() - geoPos2.latitude()) * M_PI / 180.0;
    const double longitudeArc = (geoPos1.longitude() - geoPos2.longitude()) * M_PI / 180.0;
    const double latitudeH = qPow(sin(latitudeArc * 0.5), 2);
    const double lontitudeH = qPow(sin(longitudeArc * 0.5), 2);
    const double lonFactor = cos(geoPos1.latitude() * M_PI / 180.0) * cos(geoPos2.latitude() * M_PI / 180.0);
    const double arcInRadians = 2.0 * asin(sqrt(latitudeH + lonFactor * lontitudeH));
    return mEarthRadius * arcInRadians;
}
