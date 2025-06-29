/* globals Chart:false, feather:false */

(function () {
  'use strict'

  feather.replace({ 'aria-hidden': 'true' })

  let x_label = [];
  let x_value = [];
  let y_value = [];

  $.ajax({
    url: '/api/recent',
    type: 'get',
    data : {},
    cache: true,
    dataType: 'json'
   })
   .done(function(response){
    const time_range = 50
    const max_sampling_number = time_range / response.predicted_period;
    let offset_gyro = response.ad_gyro.length - max_sampling_number;
    if (offset_gyro < 0)
    {
      offset_gyro = 0;
    }

    response.time.slice(offset_gyro).forEach((element, i)=> {
      x_label.push(i.toString());
      x_value.push(element);
    });

    response.ad_gyro.slice(offset_gyro).forEach((element, i) => {
      y_value.push(element);
    });

    let data_set = [];
    for(var i = 0 ; i < x_value.length; i ++ ) 
    {
      data_set.push({'x' : x_value[i], 'y' : y_value[i]});
    }

    let x_min = 0
    let x_max = time_range
    if(Math.min(x_value) > time_range) 
    {
      x_min = Math.min(x_value);
      x_max = x_min + time_range;
    }

    // Graphs
    var ctx = document.getElementById('myChart')
    // eslint-disable-next-line no-unused-vars
    const myChart = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [{
          data: data_set,
          lineTension: 0,
          backgroundColor: 'transparent',
          borderColor: '#007bff',
          borderWidth: 4,
          pointBackgroundColor: '#007bff'
        }]
      },
      options: {
        scales: {
          xAxes : [{
            ticks : {
              suggestedMin : x_min,
              suggestedMax : x_max
            },
            scaleLabel : {
              display : true,
              labelString : "Time [s]",
            }
          }],
          yAxes: [{
            ticks: {
              beginAtZero: false,
            },
            scaleLabel : {
              display : true,
              labelString : "Gyro Value"
            }
          }]
        },
        legend: {
          display: false
        }
      }
    })
   })
   .fail(function(xhr) {
    console.log("failed");
   });




})()
