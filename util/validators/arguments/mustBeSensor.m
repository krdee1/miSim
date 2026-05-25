function mustBeSensor(sensorModel)
    if isa(sensorModel, 'cell')
        for ii = 1:size(sensorModel, 1)
            assert(isa(sensorModel{ii}, 'sigmoidSensor', 'rfSensor'), ...
                   'Sensor in index %d is not a valid sensor class', ii);
        end
    else
        assert(isa(sensorModel, 'sigmoidSensor') || isa(sensorModel, 'rfSensor'), ...
               'Sensor is not a valid sensor class');
    end
end