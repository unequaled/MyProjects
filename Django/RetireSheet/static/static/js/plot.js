$(document).ready(function () {
    
     
    var container = document.getElementById('basic_example');
    var data = function () {
        var data_return ;
        return Handsontable.helper.createSpreadsheetData(15,12);
    };
    var settings = {data: data(),
        rowHeaders: true,
        colHeaders: true,
    };
    var hot = new Handsontable(container, settings);
 
    $.jqplot('chartdiv', [[[1, 2],[3,5.12],[5,13.1],[7,33.6],[9,85.9],[11,219.9]]]);
});
