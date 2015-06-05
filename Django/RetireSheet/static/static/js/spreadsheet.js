$(document).ready(function () {
    var container = document.getElementById('basic_example');
    var data = [
        ["Target \nROI(%)", 5 , "Inflation \nrate(%)", 0 , "Initial \nSaving",30000,"" ,"","","",""],
        ["Age", "Passive \nincome", "Saving", "Total \nincome", "Life \nExp.", "Education \nExp.", "Housing \nExp.", "Insurance \nExp.", "Total \nexp.", "Net \nincome", "Year \nend"],
        [ 25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 35, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 37, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [ 39, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
      ];
      
    yellowRenderer = function(instance, td, row, col, prop, value, cellProperties) {
        Handsontable.renderers.TextRenderer.apply(this, arguments);
        td.style.backgroundColor = 'yellow';
    };
      
    var settings = {data: data,
        rowHeaders: true,
        colHeaders: true,
        cells: function (row, col, prop) {
            if (row === 0 && (col === 1 || col === 3 || col === 5)) {
                this.renderer = yellowRenderer;
            }
            if (row === 2 && (col === 0 || col === 2)) {
                this.renderer = yellowRenderer;
            }
            if (row > 1 && (col === 4 || col === 5 || col === 6 || col === 7)) {
                this.renderer = yellowRenderer;
            }
        }
    };
    
    var hot = new Handsontable(container, settings);
    
    var plotData = function () {
        var plotData = [];
        for (var i=2; i<17; i+=1){ 
            plotData.push([data[i][0], parseFloat(data[i][10])]); 
        }
        return plotData;
    };
    
    var plot = $.jqplot('chartdiv',  [plotData()], {
        title: 'Retire Fund',
        axesDefaults: {
            labelRenderer: $.jqplot.CanvasAxisLabelRenderer
        },
          
        axes: {
            xaxis: {
                label: "Year", pad: 0
            },
            yaxis: {
                label: "Fund"
            }
        }
    });
    
    $('#chartdiv').on('shown', function(g) {
        plot.replot({ resetAxes: true });
    });
    
    var checkNum = function(){
        for (var row=0; row<17; row+=1){ 
            for (var col=0; col<8; col+=1){ 
                if( (row === 0 && (col === 1 || col === 3 || col === 5)) || 
                    (row === 2 && (col === 0 || col === 2))              ||
                    (row > 1 && (col === 4 || col === 5 || col === 6 || col === 7))
                    ){
                    if(!($.isNumeric(data[row][col]))){
                        return false;
                    }
                }
            }
        }
        return true;
    };

    $('#init').click(function(){
        $.get( "/ingredients/", function( data_in ) {
            var myObject = JSON.parse(data_in);
            for (var row=0; row<17; row+=1){ 
                for (var col=0; col<11; col+=1){ 
                    data[row][col] = myObject.data[row][col];
                }
            }
            hot.loadData(data);
            plot.destroy();
            plot.series[0].data = plotData();
            plot.replot({ resetAxes: true });
        });
    });

    $('#plot').click(function(){
        if(!checkNum())
            alert("Some entries are not number!!");
        else{
            var real_rate = parseFloat(data[0][1])-parseFloat(data[0][3]);
            var saving = parseFloat(data[0][5]);
            var age = parseFloat(data[2][0]);
            var year_save = parseFloat(data[2][2]);
            for (var row=2; row<17; row+=1){
                data[row][0] = age + row -2;
                data[row][2] = year_save;
                var exp = parseFloat(data[row][4]) + parseFloat(data[row][5])+ parseFloat(data[row][6]) + parseFloat(data[row][7]);
                
                data[row][1] = (saving*(real_rate/100)).toFixed(2);
                data[row][3] = (parseFloat(data[row][1]) + year_save).toFixed(2);
                data[row][8] = exp;
                data[row][9] = parseFloat(data[row][3]) - exp;
                saving = parseFloat(data[row][9])  + saving ;
                data[row][10] = saving.toFixed(2);
            }
            hot.render();
            plot.destroy();
            plot.series[0].data = plotData();
            plot.replot({ resetAxes: true });
        }
    });
});

