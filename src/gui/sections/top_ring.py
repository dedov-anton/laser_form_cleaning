from src.gui.sections.stub_section import StubSurfaceSection


class TopRingSection(StubSurfaceSection):
    def __init__(self, parent, app) -> None:
        super().__init__(
            parent,
            app,
            title="Верх формы (кольцо)",
            generator_id="top_ring",
            description="Генератор траектории для верхней поверхности формы — в разработке.",
        )
