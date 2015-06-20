var map;
var markers = [];

var taipei_dist = ["中正區","大同區","中山區","松山區","大安區","萬華區",
                  "信義區","士林區","北投區","內湖區","南港區","文山區"];
var new_taipei_dist = ["八里區","三芝區","三重區","三峽區","土城區","中和區",
                      "五股區","平溪區","永和區","石門區","石碇區","汐止區",
                      "坪林區","林口區","板橋區","金山區","泰山區","烏來區",
                      "貢寮區","淡水區","深坑區","新店區","新莊區","瑞芳區",
                      "萬里區","樹林區","雙溪區","蘆洲區","鶯歌區"];

function initialize() {
    var mapOptions = {
        zoom: 12,
        center: new google.maps.LatLng(25.0552, 121.5502)
    };
    map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
}  

// Sets the map on all markers in the array.
function setAllMap(map) {
  for (var i = 0; i < markers.length; i++) {
    markers[i].setMap(map);
  }
}

// Removes the markers from the map, but keeps them in the array.
function clearMarkers() {
  setAllMap(null);
}

// Shows any markers currently in the array.
function showMarkers() {
  setAllMap(map);
}

// Deletes all markers in the array by removing references to them.
function deleteMarkers() {
  clearMarkers();
  markers = [];
}

google.maps.event.addDomListener(window, 'load', initialize);

$(document).ready(function(){
    $('select#city').change(function(){ 
        $('#district').empty();
        var i =0;
        if( $(this).val() == "台北市"){
            for(; i<taipei_dist.length; i++)
                 $('#district').append('<option value='+taipei_dist[i] + '>' +taipei_dist[i] + '</option>');
        }
        else if($(this).val() == "新北市"){
            for(; i<new_taipei_dist.length; i++)
                 $('#district').append('<option value='+new_taipei_dist[i] + '>' +new_taipei_dist[i] + '</option>');
        }
        else{
             $('#district').append('<option>select</option>');
        }
    });
    
    $("#form-search").submit(function(event) {
        event.preventDefault();
        var url = "/mapsdata/" +  $("#city").val()+ "/" + $("#district").val()+ "/";
        
        deleteMarkers();
        $.get( url, function( data ) {
            
            var trash_data = JSON.parse(data);
        
            for(var i=0;i<trash_data.length;i++){
                var myLatlng = new google.maps.LatLng(parseFloat(trash_data[i].Lat),parseFloat(trash_data[i].Lng));
                
                var contentString = '<ul class="list-group">' +
                                      '<li class="list-group-item">Car Time:'+trash_data[i].CarTime +'</li>' +
                                      '<li class="list-group-item">Info:'+ trash_data[i].Info +'</li>' +  
                                    '</ul>';
                var infowindow = new google.maps.InfoWindow();
                
                var marker = new google.maps.Marker({
                    position: myLatlng,
                    map: map,
                    title: 'Ya!'
                });
                
                google.maps.event.addListener(marker,'click', (function(marker,contentString,infowindow){ 
                                                                return function() {
                                                                   infowindow.setContent(contentString);
                                                                   infowindow.open(map,marker);
                                                                };
                                                            })(marker,contentString,infowindow)); 
                                                                        
                markers.push(marker);
            }
        });
        showMarkers();
    });
});
