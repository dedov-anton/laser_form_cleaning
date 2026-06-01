from src.gui.sections.stub_section import StubSurfaceSection


class CylinderWallSection(StubSurfaceSection):
    def __init__(self, parent, app) -> None:
        super().__init__(
            parent,
            app,
            title="Поверхность качения (цилиндр)",
            generator_id="cylinder_wall",
            description="Генератор траектории для боковой поверхности цилиндра — в разработке.",
        )
