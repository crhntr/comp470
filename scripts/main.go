package main

import "fmt"

func main() {
	var (
		count, sum int
		nums       []int
	)
	for {
		var num int
		_, err := fmt.Scanf("%d\n", &num)
		if err != nil {
			break
		}
		nums = append(nums, num)
		count++
		sum += num
	}
	ave := sum / count
	fmt.Println(ave)

	s := 0
	for _, xi := range nums {
		s += (xi - ave) * (xi - ave)
	}
	math.Sqrt(s/len(nums))
	fmt.Println(std / )
}
