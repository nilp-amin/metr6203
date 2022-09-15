function LoadDataIntoWorkspace(datafile)

datafile="Pendulum2_FInallyWorking.mat"
load(datafile, 'data')
n_fields = data.numElements;

data_fields = ['flywheel_angle', 'gimbal_angle', 'pendulum_angle', 'table_angle', 'gimbal_voltage', 'gimbal motor voltage'];

simin(6) = struct();
for i = 1:n_fields
    name = data.getElement(i).Name;
    name = replace(name, ' ', '_');
    eval(sprintf("%s = data.getElement(%i).Values;", name, i))
%     save(sprintf("%s.mat", name), name)
end
end  