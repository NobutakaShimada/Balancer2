/* globals Chart:false, feather:false */

(function () {
  'use strict'

  feather.replace({ 'aria-hidden': 'true' })

  let x_label = [];
  let y_value = [];
  const max_sampling_number = 50;

  $.ajax({
    url: '/api/recent',
    type: 'get',
    data : {},
    cache: true,
    dataType: 'json'
   })
   .done(function(response){
    let offset_gyro = response.ad_gyro.length - max_sampling_number;
    if (offset_gyro < 0)
    {
      offset_gyro = 0;
    }

    response.ad_gyro.slice(offset_gyro).forEach((element, i) => {
      x_label.push(i.toString());
      y_value.push(element);
    });

    // Graphs
    var ctx = document.getElementById('myChart')
    // eslint-disable-next-line no-unused-vars
    var myChart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: x_label,
        datasets: [{
          data: y_value,
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
            scaleLabel : {
              display : true,
              labelString : "Time [s]"
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
