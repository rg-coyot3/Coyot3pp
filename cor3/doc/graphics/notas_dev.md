

* distance 2 latlon positions with altitude

source [informatix](https://www.b4x.com/android/forum/threads/distance-between-two-gps-points-with-altitude.72072/)

```
Private Sub EarthRadiusInMeters (latitudeRadians As Double) As Double
    ' latitude is geodetic, i.e. that reported by GPS
    ' http://en.wikipedia.org/wiki/Earth_radius
    Dim Const eqR As Double = 6378137.0 ' equatorial radius in meters
    Dim Const polR As Double = 6356752.3 ' polar radius in meters
    Dim CosLat As Double = Cos(latitudeRadians)
    Dim SinLat As Double = Sin(latitudeRadians)
    Dim t1 As Double = eqR * eqR * CosLat
    Dim t2 As Double = polR * polR * SinLat
    Dim t3 As Double = eqR * CosLat
    Dim t4 As Double = polR * SinLat
    Return Sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4))
End Sub

Private Sub GeocentricLatitude(Lat As Double) As Double
    ' Convert geodetic latitude 'lat' to a geocentric latitude.
    ' Geodetic latitude is the latitude As given by GPS.
    ' Geocentric latitude is the angle measured from center of Earth between a point and the equator.
    ' https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
    Dim Const e2 As Double = 0.00669437999014
    Return ATan((1.0 - e2) * Tan(Lat))
End Sub

Private Sub LocationToPoint (Latitude As Double, Longitude As Double, Altitude As Int) As Double(3)
    ' Convert (Lat, Lon, elv) To (x, y, z).
    Dim Lat As Double = Latitude * cPI / 180.0
    Dim Lon As Double = Longitude * cPI / 180.0
    Dim radius As Double = EarthRadiusInMeters(Lat)
    Dim clat As Double   = GeocentricLatitude(Lat)

    Dim cosLon As Double = Cos(Lon)
    Dim sinLon As Double = Sin(Lon)
    Dim cosLat As Double = Cos(clat)
    Dim sinLat As Double = Sin(clat)
    Dim XYZ(3) As Double
    XYZ(0) = radius * cosLon * cosLat
    XYZ(1) = radius * sinLon * cosLat
    XYZ(2) = radius * sinLat

    ' We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
    ' Now we use geodetic latitude to calculate normal vector from the surface, to correct for elevation.
    Dim cosGlat As Double = Cos(Lat)
    Dim sinGlat As Double = Sin(Lat)

    Dim nx As Double = cosGlat * cosLon
    Dim ny As Double = cosGlat * sinLon
    Dim nz As Double = sinGlat

    XYZ(0) = XYZ(0) + (Altitude * nx)
    XYZ(1) = XYZ(1) + (Altitude * ny)
    XYZ(2) = XYZ(2) + (Altitude * nz)
    Return XYZ
End Sub

Public Sub DistanceInMeters(Lat1 As Double, Long1 As Double, Alt1 As Int, Lat2 As Double, Long2 As Double, Alt2 As Int) As Double
    Dim XYZ1(3) As Double = LocationToPoint(Lat1, Long1, Alt1)
    Dim XYZ2(3) As Double = LocationToPoint(Lat2, Long2, Alt2)
    Dim dx As Double = XYZ1(0) - XYZ2(0)
    Dim dy As Double = XYZ1(1) - XYZ2(1)
    Dim dz As Double = XYZ1(2) - XYZ2(2)
    Return Sqrt (dx*dx + dy*dy + dz*dz)
End Sub
``` 