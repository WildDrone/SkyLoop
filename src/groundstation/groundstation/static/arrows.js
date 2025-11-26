// arrows.js 

var arrows = { 'initialized': false, 'feature_group': null, 'icon': null };

function init_arrows(map_id) {
    // run after leaflet is loaded
    if ( ! arrows.initialized ) {

        var map = getElement(map_id).map;       // see templates/index.html
        // map = window.app.$refs["r" + map_id].map;
        if ( ! map ) {
            console.log('Leaflet map object ' + map_id + ' not found');
            return;
        }  
        // Add markers to the map
        arrows.feature_group = L.layerGroup().addTo(map);

        arrows.initialized = true;
    }
}

function place_arrow(map_id, lat, lng, rotationAngle = 0, i, z=0) {
    init_arrows(map_id);
    if ( ! arrows.initialized ) {
        console.log('Markers not initialized / Leaflet probably not yet loaded');
        return;
    }
    //delete_arrow(0)
    // Create SVG icon as arrow and apply rotation
    var icon = L.divIcon({
        className: 'marker-icon',
        html: '<svg width="30" height="40">\
                <path d="M15 0 L0 40 L15 30 Z" fill="red"/>\
                <path d="M15 0 L15 30 L30 40 Z" fill="darkred"/>\
                </svg>',
        iconAnchor: [15, 20],
        iconSize: [30, 40],
        iconRotation: rotationAngle
    });

    // Create marker
    var arrow = L.marker([lat, lng], {
        icon: icon,
        rotationAngle: rotationAngle
    }).addTo( arrows.feature_group )
    // Rotate using CSS
    var rotationStyle = 'transform: rotate(' + rotationAngle + 'deg);';
    arrow._icon.children[0].setAttribute('style', rotationStyle);
    arrow._ix = i;
    return arrow;
}

function delete_arrow(i) {
    if ( arrows.initialized ) {
        arrows.feature_group.eachLayer(function (layer) {
            if (layer._ix == i) {
                arrows.feature_group.removeLayer(layer);
            }
        }
    )};
}

function delete_all_arrows() { 
    if ( arrows.initialized ) {
        arrows.feature_group.remove();
        arrows.feature_group = new L.featureGroup([]);
    }
}


function update_arrow_test(i, lat, lng, rotationAngle) {
    if (arrows.initialized) {
        arrows.feature_group.eachLayer(function(layer) {
            if (layer._ix == i) {
                // Mettre à jour la position
                layer.setLatLng([lat, lng]);

                // Mettre à jour la rotation
                var rotationStyle = 'transform: rotate(' + rotationAngle + 'deg);';
                layer._icon.children[0].setAttribute('style', rotationStyle);

                // Optionnellement, mettre à jour l'angle de rotation stocké
                layer.options.rotationAngle = rotationAngle;
            }
        });
    } else {
        console.log('Arrows not initialized');
    }
}

function restore_arrows(mapId, arrows) {
    arrows.forEach(arrow => {
        place_arrow(mapId, arrow.lat, arrow.lng, arrow.heading, arrow.id);
    });
}