/**
 * Draw a ball.
 *
 * Draws a ball object on a canvas. 
 *
 * @param {object}	ctx						The context of the canvas on which to draw.
 * @param {number}	cx						The x position of the ball's center point in pixels.
 * @param {number}	cy						The y position of the ball's center point in pixels.
 * @param {number}	r						The radius of the ball in pixels.
 * @param {string}	[fillStyle=sawyer_red]	The color of the ball's fill.
 */
function draw_ball(ctx,cx,cy,r,fillStyle=getComputedStyle(document.body).getPropertyValue('--sawyer-red')) {
    ctx.fillStyle = fillStyle;
    ctx.beginPath();
    ctx.arc(cx,cy,r,0,2*Math.PI,false);
    ctx.fill();
}