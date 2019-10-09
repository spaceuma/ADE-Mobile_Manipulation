require 'vizkit'
include Orocos
Orocos.initialize

gui = Vizkit.load 'gui.ui'

proxy = Orocos.name_service.get 'proxy'


table = gui.tableWidget

table.horizontalHeader().setResizeMode Qt::HeaderView.Stretch


proxy.taskStatusOut.connect_to do |data,_|

    items = table.findItems(data.name, Qt::MatchExactly)

    name = Qt::TableWidgetItem.new data.name
    status = Qt::TableWidgetItem.new "#{data.status}"

    if items.length == 0

        index = table.rowCount
    
        table.insertRow index

        table.setItem(index, 0, name)
        table.setItem(index, 1, status)

    else
        id = items.first.row
        table.setItem(id, 1, status)
    end
end



gui.show
Vizkit.exec