function params = readParams(excelFile)

    raw = readcell(excelFile);

    params = struct();

    for i = 2:size(raw,1)

        parameter = raw{i,1};
        subclass  = raw{i,2};
        value     = raw{i,3};

        params.(subclass).(parameter) = value;

    end
end