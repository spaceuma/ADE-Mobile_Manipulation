if !Orocos.initialized?
	Orocos.initialize
	Orocos.load_typekit 'base'
end

name_service = Orocos::Async::CORBA.name_service

# forward all tasks on the global name service
name_service.on_task_added do |name|
    puts "task name: #{name}"
end
