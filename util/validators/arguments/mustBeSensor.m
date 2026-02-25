function mustBeSensor(sensorModel)
    if isa(sensorModel, 'cell')
        for ii = 1:size(sensorModel, 1)
            assert(isa(sensorModel{ii}, 'sigmoidSensor'), ...
                   'Sensor in index %d is not a valid sensor class', ii);
        end
    else
        assert(isa(sensorModel, 'sigmoidSensor'), ...
               'Sensor is not a valid sensor class');
    end
end