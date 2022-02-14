// Copyright (C) 2022  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_
#define IGNITION_GAZEBO_SYSTEMS_OGRE2OCEANTILE_HH_

#include <ignition/math.hh>
#include <ignition/rendering.hh>

#include <memory>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

  class Ogre2OceanTilePrivate;

  class Ogre2OceanTile
  {
    public: virtual ~Ogre2OceanTile();

    public: Ogre2OceanTile(size_t _N, double _L, bool _hasVisuals=true);

    public: void SetWindVelocity(double _ux, double _uy);

    public: void Create();

    public: void Update(double _time);

    private: std::unique_ptr<Ogre2OceanTilePrivate> dataPtr;
  };

  typedef std::shared_ptr<Ogre2OceanTile> Ogre2OceanTilePtr;
}
}
}

#endif
