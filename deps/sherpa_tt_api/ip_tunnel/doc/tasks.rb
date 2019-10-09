include Orocos

if !Orocos.initialized?
	Orocos.initialize
	Orocos.load_typekit 'base'
end

name_service = Orocos::Async::CORBA.name_service

tasks = Orocos::Async.name_service
tasks.on_task_added do |taskName|
  tmpEmptyWaste, task_name = taskName.split('/')
  task = Orocos.name_service.get(task_name)
  ports = task.port_names
  info = Types::Paket.new
  info.taskContext = task_name
  info.inputPort = []
  ports.each do |v|
    info.inputPort.push v
    puts "Ports #{v} und Task #{task_name}"
  end
  allTasks[task_name]=info;
end
