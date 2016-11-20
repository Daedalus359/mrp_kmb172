# kmb172_p6

The bag file I used for data is in the top directory, and a slightly modified copy of the lidar transformer is in the src directory. Simply run the transformer and then the bag.

My modifications basically just suppress all command line output that is in there by default and 
add lines corresponding to the specific data I need. It is almost exactly the same otherwise.

The command line output from the transformer will put raw data relevant to the block's location in yellow and a current estimate of the block's dimensions in white.

The system keeps an updated record of the biggest and smallest x and y values that were recorded 
along with z values large enough to be confident that we are looking at the block. Also, a max z
value is recorded. These together are used to compute the dimensions of the block.

Looking at the output I got, I belive the block to be 0.417 x 0.666 x 0.180
From the yellow text, I belive that the block occupies the following region:

	x: -0.210 to 0.207, with a center right around -1.5 (take the average)
	y: -0.666 to 0.000, with a center around -0.333
	z: 0.000 to 0.180, with a center around 0.090

The total size estimates may be a little larger than the actual block because of random noise.

If I knew the noise associated with each lidar point, I could make a decent estimate of the actual
values at the edges of the block. However, I'll just use what I have.

## Example usage

## Running tests/demos
    
