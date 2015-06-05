    $('#load').click(function(){
        $.get( "/ingredients/", function( data_in ) {
            // Javascript only use "" as a string 
            var myObject = JSON.parse(data_in);
            hot.loadData(myObject.data);
        });
    });
    
    $('#save').click(function(){
        $.ajax({
            url: "/ingredients/",
            type: "POST",
            contentType: "application/json; charset=utf-8",
            data: JSON.stringify({data: hot.getData()}),
            dataType: 'text',
            success: function(result) {
                alert(result.Result);
            }
        });
    });