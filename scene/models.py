from typing import Literal

from pydantic import BaseModel, Field, model_validator


class GeoAnchor(BaseModel):
    latitude: float
    longitude: float


class GeoBounds(BaseModel):
    south: float
    west: float
    north: float
    east: float

    @model_validator(mode="after")
    def validate_order(self):
        if self.south >= self.north or self.west >= self.east:
            raise ValueError("Invalid geographic bounds")
        return self


class EnuPoint(BaseModel):
    x: float
    y: float
    z: float = 0.0


class SceneFeature(BaseModel):
    id: str
    category: Literal["building", "road", "terrain", "water"]
    footprint: list[EnuPoint] = Field(min_length=2)
    height: float = Field(default=0.0, ge=0.0)
    material: str = "itu_concrete"
    source: str = "user"


class SceneModel(BaseModel):
    schema_version: int = 1
    name: str
    anchor: GeoAnchor
    bounds: GeoBounds | None = None
    size_x: float = Field(gt=0)
    size_y: float = Field(gt=0)
    features: list[SceneFeature] = Field(default_factory=list)

