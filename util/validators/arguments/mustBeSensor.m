function mustBeSensor(sensorModel)
    validSensorModels = ["fixedCardinalSensor"; "sigmoidSensor";];
    if isa(sensorModel, "cell")
        for ii = 1:size(sensorModel, 1)
            assert(any(arrayfun(@(x) isa(sensorModel{ii}, x), validSensorModels)), "Sensor in index %d is not a valid sensor class", ii);
        end
    else
        assert(any(arrayfun(@(x) isa(sensorModel, x), validSensorModels)), "Sensor is not a valid sensor class");
    end
end