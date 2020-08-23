/**
 * Draw a bar chart.
 *
 * Draws a bar chart object on a canvas. 
 *
 * @param {object}  ctx                     The context of the canvas on which to draw.
 * @param {number}  val                     The value to display on the bar chart.
 * @param {number}  maxVal                  The maximum value displayable on the bar chart.
 * @param {number}  axisLeft                The x position of the left corner of the x-axis of the bar chart in pixels.
 * @param {number}  axisRight               The x position of the right corner of the x-axis of the bar chart in pixels.
 * @param {number}  maxHeight               The maximum height of the bar chart in pixels.
 * @param {number}  antiWidth               The distance of the edges of the bar chart from each side of the x-axis.
 * @param {string}  [fillStyle=sawyer_red]  The color of the ball's fill.
 * @param {string}  [label=null]            The label to display under the bar chart.
 */
function draw_bar(ctx, val, maxVal, axisLeft, axisRight, maxHeight, antiWidth, fillStyle=getComputedStyle(document.body).getPropertyValue('--sawyer-red'), label=null) {
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 0;
    ctx.beginPath();
    ctx.moveTo(axisLeft+antiWidth,maxHeight);
    ctx.lineTo(axisLeft+antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx.lineTo(axisRight-antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx.lineTo(axisRight-antiWidth,maxHeight);
    ctx.lineTo(axisLeft+antiWidth,maxHeight);
    ctx.fillStyle = fillStyle;
    ctx.fill();

    ctx.strokeStyle = '#FFF';
    ctx.lineWidth = 5;
    ctx.beginPath();
    ctx.moveTo(axisLeft,maxHeight);
    ctx.lineTo(axisRight,maxHeight);
    ctx.stroke();
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 0;

    if (label!==null) {
        ctx.fillText(label,axisLeft+(axisRight-axisLeft)/2-11,maxHeight+25)
    }
}